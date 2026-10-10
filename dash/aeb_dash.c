#define AEB_TELEMETRY_NO_API
#include "telemetry.h"
#include "vision_shm.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#define ALIVE_NS 1500000000ull
static const char *g_www = NULL;
static volatile sig_atomic_t g_stop;
static void on_sig(int s)
{
    (void)s;
    g_stop = 1;
}
static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}
static const telem_shm_t *g_tel;
static const vision_shm_t *g_vis;
static const vision_frame_shm_t *g_frm;
static control_shm_t *g_ctl;
static void *map_ro(const char *name, size_t sz)
{
    int fd = shm_open(name, O_RDONLY, 0);
    if (fd < 0) return NULL;
    struct stat st;
    if (fstat(fd, &st) != 0 || (size_t)st.st_size < sz) {
        close(fd);
        return NULL;
    }
    void *p = mmap(NULL, sz, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    return p == MAP_FAILED ? NULL : p;
}
static void attach(void)
{
    if (!g_tel) g_tel = (const telem_shm_t *)map_ro(TELEM_SHM_NAME, sizeof(telem_shm_t));
    if (!g_vis) g_vis = (const vision_shm_t *)map_ro(VISION_SHM_NAME, sizeof(vision_shm_t));
    if (!g_frm) g_frm = (const vision_frame_shm_t *)map_ro(VISION_FRAME_NAME, sizeof(vision_frame_shm_t));
    if (!g_ctl) {
        int fd = shm_open(CONTROL_SHM_NAME, O_RDWR | O_CREAT, 0666);
        if (fd >= 0) {
            struct stat st;
            if (fstat(fd, &st) == 0 && (size_t)st.st_size < sizeof(control_shm_t))
                if (ftruncate(fd, sizeof(control_shm_t)) != 0) {
                    close(fd);
                    return;
                }
            void *p = mmap(NULL, sizeof(control_shm_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            close(fd);
            if (p != MAP_FAILED) g_ctl = (control_shm_t *)p;
        }
    }
}
typedef struct {
    char *b;
    size_t n, cap;
} sbuf_t;
static void sb_put(sbuf_t *s, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void sb_put(sbuf_t *s, const char *fmt, ...)
{
    for (;;) {
        va_list ap;
        va_start(ap, fmt);
        int k = vsnprintf(s->b + s->n, s->cap - s->n, fmt, ap);
        va_end(ap);
        if (k < 0) return;
        if (s->n + (size_t)k < s->cap) {
            s->n += (size_t)k;
            return;
        }
        size_t nc = s->cap * 2 + (size_t)k;
        char *nb = realloc(s->b, nc);
        if (!nb) return;
        s->b = nb;
        s->cap = nc;
    }
}
static void sb_str(sbuf_t *s, const char *str, size_t maxlen)
{
    sb_put(s, "\"");
    for (size_t i = 0; i < maxlen && str[i]; i++) {
        unsigned char c = (unsigned char)str[i];
        if (c == '"' || c == '\\')
            sb_put(s, "\\%c", c);
        else if (c < 0x20)
            sb_put(s, "\\u%04x", c);
        else
            sb_put(s, "%c", c);
    }
    sb_put(s, "\"");
}
static double fin(float v)
{
    return (v == v && v < 1e9f && v > -1e9f) ? (double)v : 0.0;
}
static void json_aeb(sbuf_t *s, uint64_t now)
{
    static telem_shm_t t;
    if (!g_tel || !vision_seq_read(g_tel, &t, sizeof t, 8) || t.magic != TELEM_MAGIC ||
        t.version != TELEM_VERSION) {
        sb_put(s, "null");
        return;
    }
    uint64_t age = (t.t_ns && now >= t.t_ns) ? now - t.t_ns : UINT64_MAX;
    sb_put(s, "{\"alive\":%s,\"age_ms\":%.0f,\"uptime_s\":%.1f,\"cycle\":%u,\"pid\":%u,\"mode\":",
           age < ALIVE_NS ? "true" : "false", age == UINT64_MAX ? -1.0 : (double)age / 1e6, t.uptime_s,
           t.cycle, t.pid);
    sb_str(s, t.mode, sizeof t.mode);
    sb_put(s, ",\"state\":");
    sb_str(s, t.state, sizeof t.state);
    sb_put(s, ",\"state_id\":%u,\"risk\":", t.state_id);
    sb_str(s, t.risk, sizeof t.risk);
    sb_put(s,
           ",\"threat\":{\"valid\":%s,\"id\":%u,\"range_m\":%.3f,\"x_m\":%.3f,\"v_close\":%.3f,"
           "\"n_detected\":%u,\"n_in_corridor\":%u}",
           t.threat_valid ? "true" : "false", t.threat_id, fin(t.range_m), fin(t.x_m), fin(t.v_close_mps),
           t.n_detected, t.n_in_corridor);
    sb_put(s, ",\"ttc_valid\":%s,\"ttc_s\":%.3f,\"unavoidable\":%s,\"d_safe\":[%.3f,%.3f,%.3f]",
           t.ttc_valid ? "true" : "false", fin(t.ttc_s), t.unavoidable ? "true" : "false", fin(t.d_safe_m[0]),
           fin(t.d_safe_m[1]), fin(t.d_safe_m[2]));
    sb_put(s, ",\"motor\":{\"cmd\":");
    sb_str(s, t.motor_cmd, sizeof t.motor_cmd);
    sb_put(s, ",\"duty\":%.1f,\"backend\":", fin(t.duty_pct));
    sb_str(s, t.motor_backend, sizeof t.motor_backend);
    sb_put(s, ",\"latched\":%s,\"latency_us\":%.1f,\"max_latency_us\":%.1f}",
           t.motor_latched ? "true" : "false", fin(t.cmd_latency_us), fin(t.max_latency_us));
    sb_put(s,
           ",\"health\":{\"radar\":%s,\"imu\":%s,\"motor\":%s,\"system\":%s,\"degraded\":%s,"
           "\"guard\":%s,\"faults\":%u}",
           t.radar_ok ? "true" : "false", t.imu_ok ? "true" : "false", t.motor_ok ? "true" : "false",
           t.system_ok ? "true" : "false", t.degraded ? "true" : "false",
           t.guard_connected ? "true" : "false", t.faults);
    sb_put(s, ",\"a_long\":%.3f,\"ego\":{\"valid\":%s,\"v\":%.3f}", fin(t.a_long),
           t.ego_valid ? "true" : "false", fin(t.ego_speed_mps));
    sb_put(s,
           ",\"radar\":{\"frames_ok\":%u,\"frames_bad\":%u,\"period_ms\":%.1f,\"age_ms\":%.0f,\"tracks\":[",
           t.frames_ok, t.frames_bad, fin(t.frame_period_ms), fin(t.radar_age_ms));
    for (int i = 0; i < t.n_tracks && i < TELEM_MAX_TRACKS; i++) {
        const telem_track_t *k = &t.tracks[i];
        sb_put(
            s,
            "%s{\"id\":%u,\"x\":%.3f,\"y\":%.3f,\"r\":%.3f,\"vc\":%.3f,\"conf\":%s,\"path\":%s,\"threat\":%s}",
            i ? "," : "", k->id, fin(k->x_m), fin(k->y_m), fin(k->range_m), fin(k->v_close_mps),
            k->confirmed ? "true" : "false", k->in_corridor ? "true" : "false",
            k->is_threat ? "true" : "false");
    }
    sb_put(s, "]},\"camera\":{\"connected\":%s,\"valid\":%s,\"label\":", t.cam_connected ? "true" : "false",
           t.cam_valid ? "true" : "false");
    sb_str(s, t.cam_label, sizeof t.cam_label);
    sb_put(s, ",\"conf\":%.2f,\"bearing\":%.1f}", fin(t.cam_conf), fin(t.cam_bearing_deg));
    sb_put(s,
           ",\"config\":{\"proximity_stop_m\":%.2f,\"corridor_half_width_m\":%.2f,"
           "\"ttc\":[%.2f,%.2f,%.2f,%.2f],\"cruise_duty\":%.0f,\"auto_start\":%u,\"auto_resume\":%u}",
           fin(t.proximity_stop_m), fin(t.corridor_half_width_m), fin(t.ttc_warning_s), fin(t.ttc_partial_s),
           fin(t.ttc_strong_s), fin(t.ttc_critical_s), fin(t.cruise_duty), t.auto_start, t.auto_resume);
    sb_put(s, ",\"event_count\":%u,\"events\":[", t.event_count);
    uint32_t n = t.event_count < TELEM_MAX_EVENTS ? t.event_count : TELEM_MAX_EVENTS;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t idx = (t.event_count - n + i) % TELEM_MAX_EVENTS;
        const telem_event_t *e = &t.events[idx];
        sb_put(s, "%s{\"seq\":%u,\"t\":%.3f,\"sev\":%u,\"text\":", i ? "," : "", e->seq, e->t_s, e->sev);
        sb_str(s, e->text, sizeof e->text);
        sb_put(s, "}");
    }
    sb_put(s, "]}");
}
static void json_vision(sbuf_t *s, uint64_t now)
{
    vision_shm_t v;
    if (!g_vis || !vision_shm_read(g_vis, &v, 8) || v.magic != VISION_SHM_MAGIC ||
        v.version != VISION_SHM_VERSION) {
        sb_put(s, "null");
        return;
    }
    uint64_t age = (v.t_ns && now >= v.t_ns) ? now - v.t_ns : UINT64_MAX;
    uint64_t fno = 0;
    if (g_frm) {
        uint32_t q = __atomic_load_n(&g_frm->seq, __ATOMIC_ACQUIRE);
        if (!(q & 1u)) fno = g_frm->frame_no;
    }
    sb_put(s, "{\"alive\":%s,\"age_ms\":%.0f,\"source\":", age < ALIVE_NS ? "true" : "false",
           age == UINT64_MAX ? -1.0 : (double)age / 1e6);
    sb_str(s, v.source, sizeof v.source);
    sb_put(s,
           ",\"fps\":%.1f,\"infer_ms\":%.1f,\"frame_no\":%llu,\"jpeg_frame\":%llu,\"primary\":", fin(v.fps),
           fin(v.infer_ms), (unsigned long long)v.frame_no, (unsigned long long)fno);
    if (v.class_id > 0) {
        sb_put(s, "{\"label\":");
        sb_str(s, v.label, sizeof v.label);
        sb_put(s, ",\"conf\":%.2f,\"bearing\":%.1f,\"size\":%.3f}", fin(v.confidence), fin(v.bearing_deg),
               fin(v.area_frac));
    } else {
        sb_put(s, "null");
    }
    sb_put(s, ",\"dets\":[");
    for (uint32_t i = 0; i < v.n_det && i < VISION_MAX_DETS; i++) {
        const vision_det_t *d = &v.det[i];
        sb_put(s, "%s{\"label\":", i ? "," : "");
        sb_str(s, d->label, sizeof d->label);
        sb_put(s, ",\"conf\":%.2f,\"bearing\":%.1f,\"ahead\":%s,\"box\":[%.3f,%.3f,%.3f,%.3f]}",
               fin(d->confidence), fin(d->bearing_deg), d->ahead ? "true" : "false", fin(d->x1), fin(d->y1),
               fin(d->x2), fin(d->y2));
    }
    sb_put(s, "]}");
}
static int send_all(int fd, const void *buf, size_t n)
{
    const char *p = (const char *)buf;
    while (n > 0) {
        ssize_t k = send(fd, p, n, 0);
        if (k < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        p += k;
        n -= (size_t)k;
    }
    return 0;
}
static void reply(int fd, int code, const char *ctype, const void *body, size_t len)
{
    char hdr[256];
    const char *msg = code == 200 ? "OK" : code == 404 ? "Not Found" : code == 400 ? "Bad Request" : "Error";
    int h = snprintf(hdr, sizeof hdr,
                     "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\n"
                     "Cache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n",
                     code, msg, ctype, len);
    if (send_all(fd, hdr, (size_t)h) == 0 && len) send_all(fd, body, len);
}
static void serve_file(int fd, const char *name, const char *ctype)
{
    char path[512];
    snprintf(path, sizeof path, "%s/%s", g_www, name);
    FILE *f = fopen(path, "rb");
    if (!f) {
        char msg[640];
        int n = snprintf(msg, sizeof msg,
                         "<h2>AEB dashboard</h2><p>%s not found. Copy dash/www next to aeb_dash "
                         "or start with -w &lt;dir&gt;.</p><p>API: <a href=/api/state>/api/state</a></p>",
                         path);
        reply(fd, 404, "text/html; charset=utf-8", msg, (size_t)n);
        return;
    }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *b = malloc(n > 0 ? (size_t)n : 1);
    size_t got = b ? fread(b, 1, (size_t)(n > 0 ? n : 0), f) : 0;
    fclose(f);
    reply(fd, 200, ctype, b, got);
    free(b);
}
static void serve_state(int fd)
{
    uint64_t now = now_ns();
    attach();
    sbuf_t s = {malloc(16384), 0, 16384};
    if (!s.b) {
        reply(fd, 500, "text/plain", "oom", 3);
        return;
    }
    sb_put(&s, "{\"server_time_ms\":%.0f,\"aeb\":", (double)now / 1e6);
    json_aeb(&s, now);
    sb_put(&s, ",\"vision\":");
    json_vision(&s, now);
    sb_put(&s, "}");
    reply(fd, 200, "application/json", s.b, s.n);
    free(s.b);
}
static void serve_frame(int fd)
{
    static vision_frame_shm_t fr;
    attach();
    uint64_t now = now_ns();
    for (int tries = 0; g_frm && tries < 6; tries++) {
        uint32_t q1 = __atomic_load_n(&g_frm->seq, __ATOMIC_ACQUIRE);
        if (q1 & 1u) {
            usleep(2000);
            continue;
        }
        uint32_t sz = g_frm->size;
        if (sz == 0 || sz > VISION_JPEG_MAX) break;
        fr.t_ns = g_frm->t_ns;
        memcpy(fr.data, g_frm->data, sz);
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        if (__atomic_load_n(&g_frm->seq, __ATOMIC_RELAXED) != q1) continue;
        if (fr.t_ns == 0 || now - fr.t_ns > 5000000000ull) break;
        reply(fd, 200, "image/jpeg", fr.data, sz);
        return;
    }
    reply(fd, 404, "text/plain", "no frame", 8);
}
static void serve_cmd(int fd, const char *query)
{
    attach();
    int c = CTL_NONE;
    const char *v = query ? strstr(query, "c=") : NULL;
    if (v) {
        v += 2;
        if (!strncmp(v, "go", 2))
            c = CTL_GO;
        else if (!strncmp(v, "stop", 4))
            c = CTL_STOP;
        else if (!strncmp(v, "estop", 5))
            c = CTL_ESTOP;
        else if (!strncmp(v, "clear", 5))
            c = CTL_CLEAR;
    }
    if (c == CTL_NONE || !g_ctl) {
        const char *m = "{\"ok\":false}";
        reply(fd, 400, "application/json", m, strlen(m));
        return;
    }
    __atomic_store_n(&g_ctl->cmd, (uint32_t)c, __ATOMIC_RELAXED);
    g_ctl->t_ns = now_ns();
    g_ctl->magic = TELEM_MAGIC;
    __atomic_fetch_add(&g_ctl->seq, 1u, __ATOMIC_RELEASE);
    fprintf(stderr, "[dash] command %d sent to AEB\n", c);
    const char *m = "{\"ok\":true}";
    reply(fd, 200, "application/json", m, strlen(m));
}
static void handle_client(int fd)
{
    struct timeval tv = {2, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
    char req[4096];
    size_t n = 0;
    while (n < sizeof req - 1) {
        ssize_t k = recv(fd, req + n, sizeof req - 1 - n, 0);
        if (k <= 0) break;
        n += (size_t)k;
        req[n] = '\0';
        if (strstr(req, "\r\n\r\n")) break;
    }
    req[n] = '\0';
    char method[8] = "", target[512] = "";
    if (sscanf(req, "%7s %511s", method, target) != 2) {
        reply(fd, 400, "text/plain", "bad", 3);
        return;
    }
    char *query = strchr(target, '?');
    if (query) *query++ = '\0';
    if (!strcmp(target, "/") || !strcmp(target, "/index.html"))
        serve_file(fd, "index.html", "text/html; charset=utf-8");
    else if (!strcmp(target, "/api/state"))
        serve_state(fd);
    else if (!strcmp(target, "/api/frame.jpg"))
        serve_frame(fd);
    else if (!strcmp(target, "/api/cmd"))
        serve_cmd(fd, query);
    else
        reply(fd, 404, "text/plain", "not found", 9);
}
int main(int argc, char **argv)
{
    int port = 8080, opt;
    static char www_default[512];
    snprintf(www_default, sizeof www_default, "%s", argv[0]);
    char *slash = strrchr(www_default, '/');
    if (slash)
        snprintf(slash, sizeof www_default - (size_t)(slash - www_default), "/www");
    else
        snprintf(www_default, sizeof www_default, "www");
    g_www = www_default;
    while ((opt = getopt(argc, argv, "p:w:h")) != -1) {
        switch (opt) {
        case 'p': port = atoi(optarg); break;
        case 'w': g_www = optarg; break;
        default: printf("usage: %s [-p port(8080)] [-w www-dir]\n", argv[0]); return opt == 'h' ? 0 : 1;
        }
    }
    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    struct sched_param sp;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = 9;
    pthread_setschedparam(pthread_self(), SCHED_RR, &sp);
    int ls = socket(AF_INET, SOCK_STREAM, 0);
    if (ls < 0) {
        perror("socket");
        return 1;
    }
    int one = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons((uint16_t)port);
    for (int tries = 0; bind(ls, (struct sockaddr *)&a, sizeof a) != 0; tries++) {
        if (tries == 0)
            fprintf(stderr, "[dash] port %d busy (%s) - another aeb_dash running? retrying...\n", port,
                    strerror(errno));
        if (tries >= 15 || g_stop) {
            perror("bind");
            return 1;
        }
        sleep(2);
    }
    if (listen(ls, 16) != 0) {
        perror("listen");
        return 1;
    }
    attach();
    printf("[dash] AEB dashboard on http://<this-pi-ip>:%d/  (www: %s)\n", port, g_www);
    printf("[dash] telemetry %s, camera %s\n", g_tel ? "attached" : "waiting for aeb",
           g_vis ? "attached" : "waiting for aeb_vision");
    fflush(stdout);
    while (!g_stop) {
        struct sockaddr_in c;
        socklen_t cl = sizeof c;
        int fd = accept(ls, (struct sockaddr *)&c, &cl);
        if (fd < 0) {
            if (errno == EINTR) continue;
            perror("accept");
            usleep(100000);
            continue;
        }
        handle_client(fd);
        close(fd);
    }
    close(ls);
    printf("[dash] stopped\n");
    return 0;
}
