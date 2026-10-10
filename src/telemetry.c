#include "telemetry.h"
#include "shared_state.h"
#include "motor_bts7960.h"
#include "watchdog.h"
#include "platform_qnx.h"
#include "params.h"
#include "config.h"
#include "camera_if.h"
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
static telem_shm_t *g_shm;
static telem_shm_t g_stage;
static control_shm_t *g_ctl;
static uint32_t g_ctl_seen;
static uint64_t g_t0;
static struct {
    uint32_t seq;
    camera_det_t d;
    uint8_t link;
} g_cam;
static void *open_shm(const char *name, size_t sz, int *created)
{
    int fd = shm_open(name, O_RDWR | O_CREAT, 0666);
    if (fd < 0) return NULL;
    fchmod(fd, 0666);
    struct stat st;
    *created = (fstat(fd, &st) == 0 && (size_t)st.st_size < sz);
    if (*created && ftruncate(fd, (off_t)sz) != 0) {
        close(fd);
        return NULL;
    }
    void *p = mmap(NULL, sz, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    return p == MAP_FAILED ? NULL : p;
}
static void seq_begin(uint32_t *seq)
{
    uint32_t q = __atomic_load_n(seq, __ATOMIC_RELAXED);
    __atomic_store_n(seq, q + 1u, __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_RELEASE);
}
static void seq_end(uint32_t *seq)
{
    uint32_t q = __atomic_load_n(seq, __ATOMIC_RELAXED);
    __atomic_store_n(seq, q + 1u, __ATOMIC_RELEASE);
}
int telemetry_init(const char *mode_name)
{
    int created;
    g_t0 = plat_now_ns();
    memset(&g_stage, 0, sizeof g_stage);
    g_stage.magic = TELEM_MAGIC;
    g_stage.version = TELEM_VERSION;
    g_stage.pid = (uint32_t)getpid();
    snprintf(g_stage.mode, sizeof g_stage.mode, "%s", mode_name ? mode_name : "?");
    snprintf(g_stage.state, sizeof g_stage.state, "STARTING");
    snprintf(g_stage.motor_cmd, sizeof g_stage.motor_cmd, "DISABLE");
    g_shm = (telem_shm_t *)open_shm(TELEM_SHM_NAME, sizeof(telem_shm_t), &created);
    g_ctl = (control_shm_t *)open_shm(CONTROL_SHM_NAME, sizeof(control_shm_t), &created);
    if (g_ctl) {
        if (g_ctl->magic != TELEM_MAGIC) {
            g_ctl->magic = TELEM_MAGIC;
            g_ctl->cmd = CTL_NONE;
        }
        g_ctl_seen = __atomic_load_n(&g_ctl->seq, __ATOMIC_ACQUIRE);
    }
    return (g_shm && g_ctl) ? 0 : -1;
}
void telemetry_cycle(const aeb_status_t *s, motor_cmd_t cmd, float lat_us)
{
    telem_shm_t *t = &g_stage;
    t->cycle = s->cycle;
    t->state_id = (uint8_t)s->state;
    t->risk_id = (uint8_t)s->risk;
    snprintf(t->state, sizeof t->state, "%s", aeb_state_name(s->state));
    snprintf(t->risk, sizeof t->risk, "%s", risk_name(s->risk));
    t->threat_valid = s->threat.valid;
    t->threat_id = s->threat.id;
    t->range_m = s->threat.range_m;
    t->x_m = s->threat.x_m;
    t->v_close_mps = s->threat.v_close_mps;
    t->n_detected = s->threat.n_detected;
    t->n_in_corridor = s->threat.n_in_corridor;
    t->ttc_valid = s->ttc_valid;
    t->ttc_s = s->ttc_s;
    t->unavoidable = s->unavoidable;
    for (int i = 0; i < 3; i++) t->d_safe_m[i] = s->d_safe_m[i] > 99.0f ? 99.0f : s->d_safe_m[i];
    t->radar_ok = s->radar_ok;
    t->imu_ok = s->imu_ok;
    t->motor_ok = s->motor_ok;
    t->system_ok = s->system_ok;
    t->degraded = s->degraded;
    t->faults = s->faults;
    t->a_long = s->a_long;
    t->ego_valid = s->ego_valid;
    t->ego_speed_mps = s->ego_speed_mps;
    snprintf(t->motor_cmd, sizeof t->motor_cmd, "%s", motor_cmd_name(cmd.type));
    t->duty_pct = cmd.type == MOTOR_DRIVE ? cmd.duty_pct : 0.0f;
    t->cmd_latency_us = lat_us;
    if (lat_us > t->max_latency_us) t->max_latency_us = lat_us;
}
void telemetry_event(double t_s, int sev, const char *text)
{
    telem_shm_t *t = &g_stage;
    telem_event_t *e = &t->events[t->event_count % TELEM_MAX_EVENTS];
    t->event_count++;
    e->seq = t->event_count;
    e->sev = (uint8_t)sev;
    e->t_s = t_s;
    snprintf(e->text, sizeof e->text, "%s", text);
}
void telemetry_camera(const camera_det_t *d, int link)
{
    seq_begin(&g_cam.seq);
    g_cam.d = *d;
    g_cam.link = (uint8_t)link;
    seq_end(&g_cam.seq);
}
void telemetry_publish(void)
{
    if (!g_shm) return;
    telem_shm_t *t = &g_stage;
    uint64_t now = plat_now_ns();
    t->t_ns = now;
    t->uptime_s = (double)(now - g_t0) / 1e9;
    radar_state_t r;
    shared_radar_get(&r);
    t->frames_ok = r.frames_ok;
    t->frames_bad = r.frames_bad;
    t->frame_period_ms = r.frame_period_ms;
    t->radar_age_ms = r.t_ns ? (float)((double)age_ns(now, r.t_ns) / 1e6) : -1.0f;
    int n = r.n_tracks < TELEM_MAX_TRACKS ? r.n_tracks : TELEM_MAX_TRACKS;
    t->n_tracks = (uint8_t)n;
    for (int i = 0; i < n; i++) {
        const radar_track_t *k = &r.tracks[i];
        telem_track_t *o = &t->tracks[i];
        o->id = k->id;
        o->confirmed = k->confirmed;
        o->in_corridor = k->in_corridor;
        o->is_threat = t->threat_valid && k->id == t->threat_id;
        o->x_m = k->x_m;
        o->y_m = k->y_m;
        o->range_m = k->range_m;
        o->v_close_mps = k->v_close_mps;
    }
    for (int tries = 0; tries < 4; tries++) {
        uint32_t q1 = __atomic_load_n(&g_cam.seq, __ATOMIC_ACQUIRE);
        if (q1 & 1u) continue;
        camera_det_t d = g_cam.d;
        uint8_t link = g_cam.link;
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        if (__atomic_load_n(&g_cam.seq, __ATOMIC_RELAXED) != q1) continue;
        t->cam_connected = link;
        t->cam_valid = d.valid;
        snprintf(t->cam_label, sizeof t->cam_label, "%s", d.valid ? d.label : "");
        t->cam_conf = d.confidence;
        t->cam_bearing_deg = d.bearing_deg;
        t->cam_fps = d.fps;
        t->cam_infer_ms = d.infer_ms;
        break;
    }
    snprintf(t->motor_backend, sizeof t->motor_backend, "%s", motor_backend_name(motor_backend()));
    t->motor_latched = motor_safe_latched();
    t->guard_connected = watchdog_guard_connected();
    t->proximity_stop_m = g_params.proximity_stop_m;
    t->corridor_half_width_m = g_params.corridor_half_width_m;
    t->ttc_warning_s = g_params.ttc_warning_s;
    t->ttc_partial_s = g_params.ttc_partial_s;
    t->ttc_strong_s = g_params.ttc_strong_s;
    t->ttc_critical_s = g_params.ttc_critical_s;
    t->cruise_duty = g_params.cruise_duty;
    t->auto_start = (uint8_t)g_params.auto_start;
    t->auto_resume = (uint8_t)g_params.auto_resume;
    seq_begin(&g_shm->seq);
    memcpy((char *)g_shm + sizeof(uint32_t), (const char *)t + sizeof(uint32_t),
           sizeof(telem_shm_t) - sizeof(uint32_t));
    seq_end(&g_shm->seq);
}
int control_poll(void)
{
    if (!g_ctl) return CTL_NONE;
    uint32_t q = __atomic_load_n(&g_ctl->seq, __ATOMIC_ACQUIRE);
    if (q == g_ctl_seen) return CTL_NONE;
    g_ctl_seen = q;
    uint32_t c = __atomic_load_n(&g_ctl->cmd, __ATOMIC_RELAXED);
    return (c >= CTL_GO && c <= CTL_CLEAR) ? (int)c : CTL_NONE;
}
