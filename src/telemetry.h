#ifndef AEB_TELEMETRY_H
#define AEB_TELEMETRY_H
#include <stdint.h>
#define TELEM_SHM_NAME "/aeb_telemetry"
#define CONTROL_SHM_NAME "/aeb_control"
#define TELEM_MAGIC 0x41454254u
#define TELEM_VERSION 1u
#define TELEM_MAX_TRACKS 8
#define TELEM_MAX_EVENTS 32
#define TELEM_EVENT_LEN 112
typedef struct {
    uint8_t id, confirmed, in_corridor, is_threat;
    float x_m, y_m, range_m, v_close_mps;
} telem_track_t;
typedef struct {
    uint32_t seq;
    uint8_t sev;
    uint8_t pad[3];
    double t_s;
    char text[TELEM_EVENT_LEN];
} telem_event_t;
typedef struct {
    uint32_t seq;
    uint32_t magic, version, pid;
    uint64_t t_ns;
    double uptime_s;
    uint32_t cycle;
    char mode[12];
    char state[20];
    char risk[10];
    uint8_t state_id, risk_id;
    uint8_t threat_valid, threat_id;
    float range_m, x_m, v_close_mps;
    uint8_t ttc_valid, unavoidable, n_detected, n_in_corridor;
    float ttc_s;
    float d_safe_m[3];
    char motor_cmd[10];
    char motor_backend[20];
    float duty_pct;
    float cmd_latency_us;
    float max_latency_us;
    uint8_t radar_ok, imu_ok, motor_ok, system_ok, degraded, guard_connected, motor_latched, pad0;
    uint32_t faults;
    float a_long;
    float ego_speed_mps;
    uint8_t ego_valid, pad1[3];
    uint32_t frames_ok, frames_bad;
    float frame_period_ms;
    float radar_age_ms;
    uint8_t n_tracks, pad2[3];
    telem_track_t tracks[TELEM_MAX_TRACKS];
    uint8_t cam_connected, cam_valid, pad3[2];
    char cam_label[16];
    float cam_conf, cam_bearing_deg, cam_fps, cam_infer_ms;
    float proximity_stop_m, corridor_half_width_m;
    float ttc_warning_s, ttc_partial_s, ttc_strong_s, ttc_critical_s;
    float cruise_duty;
    uint8_t auto_start, auto_resume, pad4[2];
    uint32_t event_count;
    telem_event_t events[TELEM_MAX_EVENTS];
} telem_shm_t;
enum { CTL_NONE = 0, CTL_GO = 1, CTL_STOP = 2, CTL_ESTOP = 3, CTL_CLEAR = 4 };
typedef struct {
    uint32_t seq;
    uint32_t magic;
    uint32_t cmd;
    uint32_t pad;
    uint64_t t_ns;
} control_shm_t;
#ifndef AEB_TELEMETRY_NO_API
#include "aeb_types.h"
#include "camera_if.h"
int telemetry_init(const char *mode_name);
void telemetry_cycle(const aeb_status_t *st, motor_cmd_t cmd, float cmd_latency_us);
void telemetry_event(double t_s, int sev, const char *text);
void telemetry_publish(void);
void telemetry_camera(const camera_det_t *d, int link);
int control_poll(void);
#endif
#endif
