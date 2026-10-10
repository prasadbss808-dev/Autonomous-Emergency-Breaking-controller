#include "logger.h"
#include "shared_state.h"
#include "platform_qnx.h"
#include "params.h"
#include "config.h"
#include "telemetry.h"
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <sys/slog2.h>
#define RING_MASK (LOG_RING_SIZE - 1u)
_Static_assert((LOG_RING_SIZE & (LOG_RING_SIZE - 1)) == 0, "LOG_RING_SIZE must be a power of 2");
typedef struct {
    uint64_t t_ns;
    uint8_t kind;
    uint8_t sev;
    aeb_status_t st;
    motor_cmd_t cmd;
    float cmd_latency_us;
    char text[112];
} log_rec_t;
typedef struct {
    atomic_uint seq;
    log_rec_t rec;
} slot_t;
static slot_t g_ring[LOG_RING_SIZE];
static atomic_uint g_head;
static unsigned g_tail;
static atomic_uint g_dropped;
static FILE *g_csv;
static bool g_console;
static uint64_t g_t0;
static slog2_buffer_t g_slog;
static bool g_slog_ok;
static log_rec_t g_last_cycle;
static bool g_have_cycle;
static bool ring_push(const log_rec_t *r)
{
    unsigned pos = atomic_load_explicit(&g_head, memory_order_relaxed);
    for (;;) {
        slot_t *s = &g_ring[pos & RING_MASK];
        unsigned seq = atomic_load_explicit(&s->seq, memory_order_acquire);
        int diff = (int)(seq - pos);
        if (diff == 0) {
            if (atomic_compare_exchange_weak_explicit(&g_head, &pos, pos + 1, memory_order_relaxed,
                                                      memory_order_relaxed)) {
                s->rec = *r;
                atomic_store_explicit(&s->seq, pos + 1, memory_order_release);
                return true;
            }
        } else if (diff < 0) {
            atomic_fetch_add(&g_dropped, 1u);
            return false;
        } else {
            pos = atomic_load_explicit(&g_head, memory_order_relaxed);
        }
    }
}
static bool ring_pop(log_rec_t *out)
{
    slot_t *s = &g_ring[g_tail & RING_MASK];
    unsigned seq = atomic_load_explicit(&s->seq, memory_order_acquire);
    if ((int)(seq - (g_tail + 1)) < 0) return false;
    *out = s->rec;
    atomic_store_explicit(&s->seq, g_tail + LOG_RING_SIZE, memory_order_release);
    g_tail++;
    return true;
}
unsigned log_dropped(void)
{
    return atomic_load(&g_dropped);
}
int log_init(const char *csv_path, bool console_status)
{
    for (unsigned i = 0; i < LOG_RING_SIZE; i++) atomic_store(&g_ring[i].seq, i);
    g_t0 = plat_now_ns();
    g_console = console_status;
    slog2_buffer_set_config_t cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.buffer_set_name = SLOG_BUFFER_NAME;
    cfg.num_buffers = 1;
    cfg.verbosity_level = SLOG2_INFO;
    cfg.buffer_config[0].buffer_name = "events";
    cfg.buffer_config[0].num_pages = 8;
    g_slog_ok = (slog2_register(&cfg, &g_slog, 0) == 0);
    if (!g_slog_ok) fprintf(stderr, "[log] slog2_register failed - slogger2 logging disabled\n");
    if (csv_path && csv_path[0]) {
        g_csv = fopen(csv_path, "w");
        if (!g_csv) {
            perror("[log] fopen CSV");
            return -1;
        }
        fprintf(g_csv, "t_ms,kind,state,risk,target_valid,target_id,range_m,x_m,v_close_mps,"
                       "ttc_s,d_safe_partial_m,d_safe_strong_m,d_safe_full_m,ego_mps,ego_valid,"
                       "a_long_mps2,motor_cmd,duty_pct,cmd_latency_us,radar_ok,imu_ok,motor_ok,"
                       "system_ok,faults_hex,event\n");
    }
    return 0;
}
void log_event(int sev, const char *fmt, ...)
{
    log_rec_t r;
    memset(&r, 0, sizeof r);
    r.t_ns = plat_now_ns();
    r.kind = 1;
    r.sev = (uint8_t)sev;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(r.text, sizeof r.text, fmt, ap);
    va_end(ap);
    ring_push(&r);
}
void log_cycle(const aeb_status_t *st, motor_cmd_t cmd, float cmd_latency_us)
{
    log_rec_t r;
    r.t_ns = st->t_ns;
    r.kind = 0;
    r.sev = 0;
    r.st = *st;
    r.cmd = cmd;
    r.cmd_latency_us = cmd_latency_us;
    r.text[0] = '\0';
    ring_push(&r);
}
static const char *sev_name(int s)
{
    static const char *const n[] = {"INFO", "WARN", "ERROR", "CRIT"};
    return (s >= 0 && s <= 3) ? n[s] : "?";
}
static void write_csv(const log_rec_t *r)
{
    if (!g_csv) return;
    double t_ms = (double)(r->t_ns - g_t0) / 1e6;
    if (r->kind == 1) {
        fprintf(g_csv, "%.3f,EVENT,,,,,,,,,,,,,,,,,,,,,,,\"%s %s\"\n", t_ms, sev_name(r->sev), r->text);
        return;
    }
    const aeb_status_t *s = &r->st;
    fprintf(
        g_csv,
        "%.3f,CYCLE,%s,%s,%d,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.3f,%s,%.1f,%.1f,%d,%d,%d,%d,0x%04x,\n",
        t_ms, aeb_state_name(s->state), risk_name(s->risk), s->threat.valid, s->threat.id,
        s->threat.valid ? s->threat.range_m : g_params.radar_clear_m, s->threat.x_m, s->threat.v_close_mps,
        s->ttc_valid ? s->ttc_s : -1.0f, s->d_safe_m[0], s->d_safe_m[1], s->d_safe_m[2],
        s->ego_valid ? s->ego_speed_mps : -1.0f, s->ego_valid, s->a_long, motor_cmd_name(r->cmd.type),
        r->cmd.duty_pct, r->cmd_latency_us, s->radar_ok, s->imu_ok, s->motor_ok, s->system_ok,
        (unsigned)s->faults);
}
static void handle(const log_rec_t *r)
{
    write_csv(r);
    if (r->kind == 1) {
        double t_s = (double)(r->t_ns - g_t0) / 1e9;
        printf("[%9.3f] %-5s %s\n", t_s, sev_name(r->sev), r->text);
        telemetry_event(t_s, r->sev, r->text);
        if (g_slog_ok) {
            uint8_t sv = r->sev == SEV_INFO    ? SLOG2_INFO
                         : r->sev == SEV_WARN  ? SLOG2_WARNING
                         : r->sev == SEV_ERROR ? SLOG2_ERROR
                                               : SLOG2_CRITICAL;
            slog2f(g_slog, 0, sv, "%s", r->text);
        }
    } else {
        g_last_cycle = *r;
        g_have_cycle = true;
        telemetry_cycle(&r->st, r->cmd, r->cmd_latency_us);
    }
}
static void print_status(void)
{
    if (!g_have_cycle) return;
    const aeb_status_t *s = &g_last_cycle.st;
    char tgt[64];
    if (s->threat.valid)
        snprintf(tgt, sizeof tgt, "id%-3u r=%5.2fm x=%+5.2f vc=%+5.2f", s->threat.id, s->threat.range_m,
                 s->threat.x_m, s->threat.v_close_mps);
    else
        snprintf(tgt, sizeof tgt, "no in-path target (det %u)        ", s->threat.n_detected);
    printf("%-15s %s ttc=%5.2f dsafeF=%4.2f a=%+5.2f %-7s %3.0f%% R%c I%c M%c S%c f=%04x\n",
           aeb_state_name(s->state), tgt, s->ttc_valid ? s->ttc_s : 99.99f,
           s->d_safe_m[2] > 99.0f ? 99.0f : s->d_safe_m[2], s->a_long, motor_cmd_name(g_last_cycle.cmd.type),
           g_last_cycle.cmd.duty_pct, s->radar_ok ? '+' : '-', s->imu_ok ? '+' : '-', s->motor_ok ? '+' : '-',
           s->system_ok ? '+' : '-', (unsigned)s->faults);
}
void log_drain_now(void)
{
    log_rec_t r;
    while (ring_pop(&r)) handle(&r);
    telemetry_publish();
    if (g_csv) fflush(g_csv);
    fflush(stdout);
}
void *logger_thread(void *arg)
{
    (void)arg;
    uint64_t next = plat_now_ns();
    uint64_t next_status = next;
    while (atomic_load(&g_running)) {
        hb_beat(HB_LOGGER);
        log_drain_now();
        uint64_t now = plat_now_ns();
        if (g_console && now >= next_status) {
            print_status();
            next_status = now + (uint64_t)CONSOLE_STATUS_MS * 1000000ull;
        }
        next += (uint64_t)LOGGER_PERIOD_MS * 1000000ull;
        if (next < now) next = now;
        plat_sleep_until_ns(next);
    }
    log_drain_now();
    return NULL;
}
void log_close(void)
{
    log_drain_now();
    if (g_csv) {
        fclose(g_csv);
        g_csv = NULL;
    }
}
