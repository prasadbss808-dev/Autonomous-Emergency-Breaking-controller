#ifndef AEB_TYPES_H
#define AEB_TYPES_H
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
typedef enum {
    AEB_NORMAL = 0,
    AEB_WARNING,
    AEB_PARTIAL_BRAKE,
    AEB_STRONG_BRAKE,
    AEB_EMERGENCY_BRAKE,
    AEB_STOPPED,
    AEB_RADAR_FAULT,
    AEB_IMU_FAULT,
    AEB_MOTOR_FAULT,
    AEB_SYSTEM_FAULT,
    AEB_STATE_COUNT
} aeb_state_t;
typedef enum { RISK_NONE = 0, RISK_WARNING, RISK_PARTIAL, RISK_STRONG, RISK_FULL } risk_level_t;
typedef enum { MOTOR_DISABLE = 0, MOTOR_DRIVE, MOTOR_BRAKE } motor_cmd_type_t;
typedef struct {
    motor_cmd_type_t type;
    float duty_pct;
} motor_cmd_t;
#define FAULT_RADAR_TIMEOUT (1u << 0)
#define FAULT_RADAR_UART (1u << 1)
#define FAULT_IMU_TIMEOUT (1u << 2)
#define FAULT_IMU_I2C (1u << 3)
#define FAULT_IMU_IMPLAUSIBLE (1u << 4)
#define FAULT_MOTOR_GPIO (1u << 5)
#define FAULT_BRAKE_NO_DECEL (1u << 6)
#define FAULT_WDOG_THREAD (1u << 7)
#define FAULT_DEADLINE_MISS (1u << 8)
#define FAULT_IMPACT (1u << 9)
#define FAULT_OPERATOR_ESTOP (1u << 10)
#define FAULT_CAMERA_LOST (1u << 11)
#define FAULT_GUARD_MISSING (1u << 12)
#define FAULTS_INFO (FAULT_CAMERA_LOST | FAULT_GUARD_MISSING)
#define FAULTS_SYSTEM (FAULT_WDOG_THREAD | FAULT_DEADLINE_MISS)
#define FAULTS_MOTOR (FAULT_MOTOR_GPIO | FAULT_BRAKE_NO_DECEL)
typedef struct {
    uint8_t id;
    uint8_t slot;
    float x_m;
    float y_m;
    float range_m;
    float v_radial_mps;
    bool doppler_valid;
    float v_rangerate_close_mps;
    bool rangerate_valid;
    float v_close_mps;
    float res_m;
    uint16_t hits, misses;
    bool confirmed;
    bool in_corridor;
    uint64_t t_last_ns;
} radar_track_t;
typedef struct {
    bool link_ok;
    uint64_t t_ns;
    uint32_t frames_ok, frames_bad, targets_rejected;
    float frame_period_ms;
    uint8_t n_tracks;
    radar_track_t tracks[RADAR_MAX_TRACKS];
} radar_state_t;
typedef struct {
    bool valid;
    uint8_t id;
    float range_m;
    float x_m;
    float v_close_mps;
    bool closing;
    uint8_t n_detected;
    uint8_t n_in_corridor;
} threat_t;
typedef struct {
    bool valid;
    uint64_t t_ns;
    float ax, ay, az;
    float gx, gy, gz;
    float a_long;
    float a_long_raw;
    float a_dyn_g;
    bool impact;
    bool calibrated;
    uint32_t samples, i2c_errors;
} imu_state_t;
typedef enum { EGO_SRC_NONE = 0, EGO_SRC_RADAR_STATIC, EGO_SRC_IMU, EGO_SRC_ENCODER, EGO_SRC_SIM } ego_src_t;
typedef struct {
    bool ego_valid;
    float ego_speed_mps;
    ego_src_t src;
    bool stopped;
    uint64_t t_ns;
} vehicle_state_t;
typedef struct {
    uint64_t t_ns;
    uint32_t cycle;
    aeb_state_t state;
    risk_level_t risk;
    threat_t threat;
    bool ttc_valid;
    float ttc_s;
    float d_safe_m[3];
    bool unavoidable;
    bool radar_ok, imu_ok, motor_ok, system_ok;
    bool degraded;
    uint32_t faults;
    float ego_speed_mps;
    bool ego_valid;
    float a_long;
} aeb_status_t;
static inline uint64_t age_ns(uint64_t now_ns, uint64_t t_ns)
{
    return (now_ns > t_ns) ? (now_ns - t_ns) : 0u;
}
const char *aeb_state_name(aeb_state_t s);
const char *risk_name(risk_level_t r);
const char *motor_cmd_name(motor_cmd_type_t t);
#endif
