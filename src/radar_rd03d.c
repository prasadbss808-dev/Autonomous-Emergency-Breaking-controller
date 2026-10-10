#include "radar_rd03d.h"
#include "rd03d_proto.h"
#include "radar_track.h"
#include "params.h"
#include "config.h"
#include "shared_state.h"
#include "platform_qnx.h"
#include "logger.h"
#include "ipc.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/neutrino.h>
static int g_fd = -1;
static char g_dev[64];
static int g_baud;
static rd03d_parser_t g_ps;
static tracker_t g_tk;
static radar_state_t g_rs;
static bool g_verbose_radar;
static bool g_hexdump;
static uint32_t g_rx_bytes;
void radar_set_hexdump(bool v)
{
    g_hexdump = v;
}
static uint64_t g_last_frame_ns;
void radar_set_verbose(bool v)
{
    g_verbose_radar = v;
}
static int uart_config(int fd, int baud)
{
    struct termios t;
    if (tcgetattr(fd, &t) != 0) {
        perror("[radar] tcgetattr");
        return -1;
    }
    t.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF);
    t.c_oflag &= ~OPOST;
    t.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    t.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
#ifdef IHFLOW
    t.c_cflag &= ~(IHFLOW | OHFLOW);
#endif
#ifdef CRTSCTS
    t.c_cflag &= ~CRTSCTS;
#endif
    t.c_cflag |= CS8 | CREAD | CLOCAL;
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    if (cfsetispeed(&t, (speed_t)baud) != 0 || cfsetospeed(&t, (speed_t)baud) != 0) {
        perror("[radar] cfset*speed");
        return -1;
    }
    if (tcsetattr(fd, TCSANOW, &t) != 0) {
        perror("[radar] tcsetattr");
        return -1;
    }
    struct termios chk;
    if (tcgetattr(fd, &chk) == 0 && (int)cfgetispeed(&chk) != baud) {
        fprintf(stderr, "[radar] driver did not accept %d baud (reports %d)\n", baud, (int)cfgetispeed(&chk));
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return 0;
}
int radar_init(const char *dev, int baud)
{
    rd03d_parser_init(&g_ps);
    tracker_init(&g_tk);
    memset(&g_rs, 0, sizeof g_rs);
    g_last_frame_ns = 0;
    if (dev == NULL) {
        g_rs.link_ok = true;
        return 0;
    }
    strncpy(g_dev, dev, sizeof g_dev - 1);
    g_baud = baud;
    g_fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (g_fd < 0) {
        fprintf(stderr, "[radar] open %s failed: %s\n", dev, strerror(errno));
        fault_set(FAULT_RADAR_UART);
        return -1;
    }
    if (uart_config(g_fd, baud) != 0) {
        close(g_fd);
        g_fd = -1;
        fault_set(FAULT_RADAR_UART);
        return -1;
    }
#if RADAR_SEND_MULTI_TARGET_CMD
    uint8_t cmd[RD03D_CMD_FRAME_LEN];
    size_t n = rd03d_build_cmd(RD03D_CMD_MULTI, cmd, sizeof cmd);
    plat_sleep_ms(100);
    if (write(g_fd, cmd, n) != (ssize_t)n)
        fprintf(stderr, "[radar] warning: multi-target command write failed: %s\n", strerror(errno));
    tcdrain(g_fd);
#endif
    g_rs.link_ok = true;
    return 0;
}
void radar_close(void)
{
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
}
static void print_frame(const radar_state_t *rs)
{
    printf("[radar] frame #%u period=%.1fms rejected=%u bad=%u tracks=%u\n", (unsigned)rs->frames_ok,
           rs->frame_period_ms, (unsigned)rs->targets_rejected, (unsigned)rs->frames_bad, rs->n_tracks);
    for (int i = 0; i < rs->n_tracks; i++) {
        const radar_track_t *t = &rs->tracks[i];
        printf("        id%-3u slot%u x=%+6.2f y=%6.2f r=%6.2f v_rad=%+5.2f v_close=%+5.2f %s %s hits=%u\n",
               t->id, t->slot, t->x_m, t->y_m, t->range_m, t->v_radial_mps, t->v_close_mps,
               t->confirmed ? "CONF" : "tent", t->in_corridor ? "IN-PATH" : "beside ", t->hits);
    }
}
void radar_ingest(const uint8_t *buf, int n, uint64_t t_rx_ns)
{
    for (int i = 0; i < n; i++) {
        rd03d_frame_t f;
        if (!rd03d_parser_feed(&g_ps, buf[i], &f)) continue;
        radar_det_t d[RD03D_NUM_SLOTS];
        int nd = radar_frame_to_detections(&f, &g_params, d, &g_rs.targets_rejected);
        tracker_update(&g_tk, d, nd, t_rx_ns, &g_params);
        g_rs.n_tracks = (uint8_t)tracker_export(&g_tk, g_rs.tracks, RADAR_MAX_TRACKS);
        if (g_last_frame_ns) {
            float per = (float)(t_rx_ns - g_last_frame_ns) / 1e6f;
            g_rs.frame_period_ms = g_rs.frame_period_ms > 0 ? 0.9f * g_rs.frame_period_ms + 0.1f * per : per;
        }
        g_last_frame_ns = t_rx_ns;
        g_rs.t_ns = t_rx_ns;
        g_rs.frames_ok = g_ps.frames_ok;
        g_rs.frames_bad = g_ps.bad_footer;
        g_rs.link_ok = true;
        shared_radar_put(&g_rs);
        if (g_verbose_radar) print_frame(&g_rs);
    }
}
void radar_housekeeping(uint64_t now_ns)
{
    static uint64_t last_pub;
    if (now_ns - last_pub > 100000000ull) {
        g_rs.frames_bad = g_ps.bad_footer;
        shared_radar_put(&g_rs);
        last_pub = now_ns;
    }
}
void *radar_thread(void *arg)
{
    (void)arg;
    int chid, coid;
    timer_t tid;
    if (plat_channel_create(&chid, &coid) != 0 ||
        plat_timer_periodic(coid, PC_TIMER, PRIO_RADAR, RADAR_POLL_MS * 1000u, &tid) != 0) {
        log_event(SEV_CRIT, "radar thread: channel/timer creation failed (%s)", strerror(errno));
        fault_set(FAULT_RADAR_UART);
        return NULL;
    }
    uint8_t buf[512];
    uint64_t last_reopen = 0, last_stat = plat_now_ns();
    uint32_t last_bytes = 0;
    bool was_ok = false;
    while (atomic_load(&g_running)) {
        struct _pulse p;
        if (MsgReceivePulse(chid, &p, sizeof p, NULL) == -1) continue;
        if (p.code != PC_TIMER) continue;
        hb_beat(HB_RADAR);
        uint64_t now = plat_now_ns();
        if (g_fd < 0) {
            g_rs.link_ok = false;
            shared_radar_put(&g_rs);
            if (now - last_reopen > 1000000000ull) {
                last_reopen = now;
                if (radar_init(g_dev, g_baud) == 0) log_event(SEV_WARN, "radar UART %s re-opened", g_dev);
            }
            continue;
        }
        for (int k = 0; k < 8; k++) {
            ssize_t n = read(g_fd, buf, sizeof buf);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) break;
                log_event(SEV_CRIT, "radar UART read error: %s", strerror(errno));
                fault_set(FAULT_RADAR_UART);
                close(g_fd);
                g_fd = -1;
                break;
            }
            if (n == 0) break;
            g_rx_bytes += (uint32_t)n;
            if (g_hexdump) {
                printf("[uart] %3d bytes:", (int)n);
                for (ssize_t i = 0; i < n && i < 64; i++) printf(" %02X", buf[i]);
                printf("%s\n", n > 64 ? " ..." : "");
            }
            radar_ingest(buf, (int)n, plat_now_ns());
            if (n < (ssize_t)sizeof buf) break;
        }
        if (g_verbose_radar && now - last_stat >= 1000000000ull) {
            printf("[radar] last 1 s: %u bytes received, frames ok=%u bad=%u%s\n",
                   (unsigned)(g_rx_bytes - last_bytes), (unsigned)g_ps.frames_ok, (unsigned)g_ps.bad_footer,
                   g_rx_bytes == last_bytes
                       ? "   <-- NO DATA from radar (check TX/RX crossed, radar 5V, msix-rp1, UART device)"
                       : "");
            last_bytes = g_rx_bytes;
            last_stat = now;
        }
        radar_housekeeping(now);
        bool ok =
            g_rs.t_ns && age_ns(plat_now_ns(), g_rs.t_ns) <= (uint64_t)g_params.radar_timeout_ms * 1000000ull;
        if (ok != was_ok) {
            log_event(ok ? SEV_INFO : SEV_ERROR, "radar data %s (frames ok=%u bad=%u)",
                      ok ? "FRESH" : "STALE/TIMEOUT", (unsigned)g_rs.frames_ok, (unsigned)g_rs.frames_bad);
            was_ok = ok;
        }
    }
    timer_delete(tid);
    plat_channel_destroy(chid, coid);
    return NULL;
}
