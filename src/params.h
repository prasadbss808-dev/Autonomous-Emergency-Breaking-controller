#ifndef AEB_PARAMS_H
#define AEB_PARAMS_H
#include <stdbool.h>
typedef struct {
    int pin_front_rpwm, pin_front_lpwm, pin_front_en;
    int pin_rear_rpwm, pin_rear_lpwm, pin_rear_en;
    float radar_min_range_m, radar_max_range_m, radar_clear_m;
    float radar_max_lateral_m, radar_max_speed_mps;
    int radar_drop_sentinel_speed;
    int radar_timeout_ms;
    float track_gate_m;
    int track_confirm_hits, track_max_misses;
    float track_range_alpha, track_vel_alpha;
    int imu_timeout_ms;
    int imu_forward_axis, imu_forward_sign;
    float imu_lpf_alpha;
    float impact_threshold_g;
    int impact_debounce_n;
    float ttc_warning_s, ttc_partial_s, ttc_strong_s, ttc_critical_s;
    float decel_model[3];
    float decel_measured[3];
    float safety_factor, t_delay_s;
    float partial_margin_k, warning_margin_k;
    float proximity_stop_m;
    float v_close_min_mps;
    float corridor_half_width_m;
    float hyst_factor;
    int downgrade_hold_ms, emergency_min_hold_ms, emergency_max_hold_ms;
    int auto_resume;
    int auto_start;
    int imu_fault_policy;
    float degraded_max_duty;
    float cruise_duty, warning_duty, partial_duty, strong_duty;
    float motor_max_duty;
    int motor_axles;
    int pwm_hz, pwm_hz_fallback;
    int verify_delay_ms, verify_window_ms;
    float verify_min_speed_mps;
    float verify_min_decel[3];
    float v_stopped_mps, stop_quiet_accel;
    int stop_quiet_ms;
    int watchdog_timeout_ms;
    int braking_deadline_us;
} aeb_params_t;
enum { LVL_IDX_PARTIAL = 0, LVL_IDX_STRONG = 1, LVL_IDX_FULL = 2 };
extern aeb_params_t g_params;
void params_set_defaults(aeb_params_t *p);
void params_apply_test_profile(aeb_params_t *p);
int params_load_file(aeb_params_t *p, const char *path);
float params_decel(const aeb_params_t *p, int lvl_idx);
void params_print(const aeb_params_t *p);
int params_validate(const aeb_params_t *p);
#endif
