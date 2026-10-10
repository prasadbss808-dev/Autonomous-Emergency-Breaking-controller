#include "params.h"
#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <ctype.h>
aeb_params_t g_params;
void params_set_defaults(aeb_params_t *p)
{
    memset(p, 0, sizeof(*p));
    p->pin_front_rpwm = PIN_FRONT_RPWM;
    p->pin_front_lpwm = PIN_FRONT_LPWM;
    p->pin_front_en = PIN_FRONT_EN;
    p->pin_rear_rpwm = PIN_REAR_RPWM;
    p->pin_rear_lpwm = PIN_REAR_LPWM;
    p->pin_rear_en = PIN_REAR_EN;
    p->radar_min_range_m = RADAR_MIN_RANGE_M;
    p->radar_max_range_m = RADAR_MAX_RANGE_M;
    p->radar_clear_m = RADAR_CLEAR_M;
    p->radar_max_lateral_m = RADAR_MAX_LATERAL_M;
    p->radar_max_speed_mps = RADAR_MAX_SPEED_MPS;
    p->radar_drop_sentinel_speed = RADAR_DROP_SENTINEL_SPEED;
    p->radar_timeout_ms = RADAR_TIMEOUT_MS;
    p->track_gate_m = TRACK_GATE_M;
    p->track_confirm_hits = TRACK_CONFIRM_HITS;
    p->track_max_misses = TRACK_MAX_MISSES;
    p->track_range_alpha = TRACK_RANGE_ALPHA;
    p->track_vel_alpha = TRACK_VEL_ALPHA;
    p->imu_timeout_ms = IMU_TIMEOUT_MS;
    p->imu_forward_axis = IMU_FORWARD_AXIS;
    p->imu_forward_sign = IMU_FORWARD_SIGN;
    p->imu_lpf_alpha = IMU_LPF_ALPHA;
    p->impact_threshold_g = IMPACT_THRESHOLD_G;
    p->impact_debounce_n = IMPACT_DEBOUNCE_N;
    p->ttc_warning_s = TTC_WARNING_S;
    p->ttc_partial_s = TTC_PARTIAL_S;
    p->ttc_strong_s = TTC_STRONG_S;
    p->ttc_critical_s = TTC_CRITICAL_S;
    p->decel_model[LVL_IDX_PARTIAL] = DECEL_PARTIAL;
    p->decel_model[LVL_IDX_STRONG] = DECEL_STRONG;
    p->decel_model[LVL_IDX_FULL] = DECEL_FULL;
    p->safety_factor = SAFETY_FACTOR;
    p->t_delay_s = T_DELAY_S;
    p->partial_margin_k = PARTIAL_MARGIN_K;
    p->warning_margin_k = WARNING_MARGIN_K;
    p->proximity_stop_m = PROXIMITY_STOP_M;
    p->v_close_min_mps = V_CLOSE_MIN_MPS;
    p->corridor_half_width_m = CORRIDOR_HALF_WIDTH_M;
    p->hyst_factor = HYST_FACTOR;
    p->downgrade_hold_ms = DOWNGRADE_HOLD_MS;
    p->emergency_min_hold_ms = EMERGENCY_MIN_HOLD_MS;
    p->emergency_max_hold_ms = EMERGENCY_MAX_HOLD_MS;
    p->auto_resume = AUTO_RESUME;
    p->auto_start = AUTO_START;
    p->imu_fault_policy = IMU_FAULT_POLICY;
    p->degraded_max_duty = DEGRADED_MAX_DUTY;
    p->cruise_duty = CRUISE_DUTY;
    p->warning_duty = WARNING_DUTY;
    p->partial_duty = PARTIAL_DUTY;
    p->strong_duty = STRONG_DUTY;
    p->motor_max_duty = MOTOR_MAX_DUTY;
    p->motor_axles = 3;
    p->pwm_hz = PWM_HZ;
    p->pwm_hz_fallback = PWM_HZ_FALLBACK;
    p->verify_delay_ms = VERIFY_DELAY_MS;
    p->verify_window_ms = VERIFY_WINDOW_MS;
    p->verify_min_speed_mps = VERIFY_MIN_SPEED_MPS;
    p->verify_min_decel[LVL_IDX_PARTIAL] = VERIFY_MIN_DECEL_PARTIAL;
    p->verify_min_decel[LVL_IDX_STRONG] = VERIFY_MIN_DECEL_STRONG;
    p->verify_min_decel[LVL_IDX_FULL] = VERIFY_MIN_DECEL_FULL;
    p->v_stopped_mps = V_STOPPED_MPS;
    p->stop_quiet_accel = STOP_QUIET_ACCEL;
    p->stop_quiet_ms = STOP_QUIET_MS;
    p->watchdog_timeout_ms = WATCHDOG_TIMEOUT_MS;
    p->braking_deadline_us = BRAKING_DEADLINE_US;
}
void params_apply_test_profile(aeb_params_t *p)
{
    p->auto_start = 0;
    p->imu_fault_policy = 0;
    p->degraded_max_duty = 35.0f;
    p->track_confirm_hits = 2;
    p->proximity_stop_m = 0.30f;
}
float params_decel(const aeb_params_t *p, int idx)
{
    if (idx < 0 || idx > 2) return 0.0f;
    return (p->decel_measured[idx] > 0.0f) ? p->decel_measured[idx] : p->decel_model[idx];
}
typedef enum { PT_INT, PT_FLOAT } ptype_t;
typedef struct {
    const char *key;
    ptype_t type;
    size_t off;
} pdesc_t;
#define PI_(k, f)                                                                                            \
    {                                                                                                        \
        k, PT_INT, offsetof(aeb_params_t, f)                                                                 \
    }
#define PF_(k, f)                                                                                            \
    {                                                                                                        \
        k, PT_FLOAT, offsetof(aeb_params_t, f)                                                               \
    }
static const pdesc_t k_desc[] = {
    PI_("pin_front_rpwm", pin_front_rpwm),
    PI_("pin_front_lpwm", pin_front_lpwm),
    PI_("pin_front_en", pin_front_en),
    PI_("pin_rear_rpwm", pin_rear_rpwm),
    PI_("pin_rear_lpwm", pin_rear_lpwm),
    PI_("pin_rear_en", pin_rear_en),
    PF_("radar_min_range_m", radar_min_range_m),
    PF_("radar_max_range_m", radar_max_range_m),
    PF_("radar_max_lateral_m", radar_max_lateral_m),
    PF_("radar_max_speed_mps", radar_max_speed_mps),
    PI_("radar_drop_sentinel_speed", radar_drop_sentinel_speed),
    PI_("radar_timeout_ms", radar_timeout_ms),
    PF_("track_gate_m", track_gate_m),
    PI_("track_confirm_hits", track_confirm_hits),
    PI_("track_max_misses", track_max_misses),
    PF_("track_range_alpha", track_range_alpha),
    PF_("track_vel_alpha", track_vel_alpha),
    PI_("imu_timeout_ms", imu_timeout_ms),
    PI_("imu_forward_axis", imu_forward_axis),
    PI_("imu_forward_sign", imu_forward_sign),
    PF_("imu_lpf_alpha", imu_lpf_alpha),
    PF_("impact_threshold_g", impact_threshold_g),
    PI_("impact_debounce_n", impact_debounce_n),
    PF_("ttc_warning_s", ttc_warning_s),
    PF_("ttc_partial_s", ttc_partial_s),
    PF_("ttc_strong_s", ttc_strong_s),
    PF_("ttc_critical_s", ttc_critical_s),
    PF_("decel_partial", decel_model[0]),
    PF_("decel_strong", decel_model[1]),
    PF_("decel_full", decel_model[2]),
    PF_("decel_measured_partial", decel_measured[0]),
    PF_("decel_measured_strong", decel_measured[1]),
    PF_("decel_measured_full", decel_measured[2]),
    PF_("safety_factor", safety_factor),
    PF_("t_delay_s", t_delay_s),
    PF_("partial_margin_k", partial_margin_k),
    PF_("warning_margin_k", warning_margin_k),
    PF_("proximity_stop_m", proximity_stop_m),
    PF_("v_close_min_mps", v_close_min_mps),
    PF_("corridor_half_width_m", corridor_half_width_m),
    PF_("hyst_factor", hyst_factor),
    PI_("downgrade_hold_ms", downgrade_hold_ms),
    PI_("emergency_min_hold_ms", emergency_min_hold_ms),
    PI_("emergency_max_hold_ms", emergency_max_hold_ms),
    PI_("auto_resume", auto_resume),
    PI_("auto_start", auto_start),
    PI_("imu_fault_policy", imu_fault_policy),
    PF_("degraded_max_duty", degraded_max_duty),
    PF_("cruise_duty", cruise_duty),
    PF_("warning_duty", warning_duty),
    PF_("partial_duty", partial_duty),
    PF_("strong_duty", strong_duty),
    PF_("motor_max_duty", motor_max_duty),
    PI_("motor_axles", motor_axles),
    PI_("pwm_hz", pwm_hz),
    PI_("pwm_hz_fallback", pwm_hz_fallback),
    PI_("verify_delay_ms", verify_delay_ms),
    PI_("verify_window_ms", verify_window_ms),
    PF_("verify_min_speed_mps", verify_min_speed_mps),
    PF_("verify_min_decel_partial", verify_min_decel[0]),
    PF_("verify_min_decel_strong", verify_min_decel[1]),
    PF_("verify_min_decel_full", verify_min_decel[2]),
    PF_("v_stopped_mps", v_stopped_mps),
    PF_("stop_quiet_accel", stop_quiet_accel),
    PI_("stop_quiet_ms", stop_quiet_ms),
    PI_("watchdog_timeout_ms", watchdog_timeout_ms),
    PI_("braking_deadline_us", braking_deadline_us),
};
static char *trim(char *s)
{
    while (*s && isspace((unsigned char)*s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) *--e = '\0';
    return s;
}
int params_load_file(aeb_params_t *p, const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char line[256];
    int applied = 0, lineno = 0;
    while (fgets(line, sizeof line, f)) {
        lineno++;
        char *hash = strchr(line, '#');
        if (hash) *hash = '\0';
        char *s = trim(line);
        if (*s == '\0') continue;
        char *eq = strchr(s, '=');
        if (!eq) {
            fprintf(stderr, "params: %s:%d: missing '='\n", path, lineno);
            continue;
        }
        *eq = '\0';
        char *key = trim(s), *val = trim(eq + 1);
        size_t i;
        for (i = 0; i < sizeof k_desc / sizeof k_desc[0]; i++) {
            if (strcmp(key, k_desc[i].key) == 0) break;
        }
        if (i == sizeof k_desc / sizeof k_desc[0]) {
            fprintf(stderr, "params: %s:%d: unknown key '%s' (ignored)\n", path, lineno, key);
            continue;
        }
        char *end = NULL;
        if (k_desc[i].type == PT_INT) {
            long v = strtol(val, &end, 0);
            if (end == val) {
                fprintf(stderr, "params: bad int for %s\n", key);
                continue;
            }
            *(int *)((char *)p + k_desc[i].off) = (int)v;
        } else {
            float v = strtof(val, &end);
            if (end == val) {
                fprintf(stderr, "params: bad float for %s\n", key);
                continue;
            }
            *(float *)((char *)p + k_desc[i].off) = v;
        }
        applied++;
    }
    fclose(f);
    return applied;
}
void params_print(const aeb_params_t *p)
{
    for (size_t i = 0; i < sizeof k_desc / sizeof k_desc[0]; i++) {
        if (k_desc[i].type == PT_INT)
            printf("  %-28s = %d\n", k_desc[i].key, *(const int *)((const char *)p + k_desc[i].off));
        else
            printf("  %-28s = %.3f\n", k_desc[i].key, *(const float *)((const char *)p + k_desc[i].off));
    }
}
#define CHECK(cond, msg)                                                                                     \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            fprintf(stderr, "params: INVALID: %s\n", msg);                                                   \
            bad++;                                                                                           \
        }                                                                                                    \
    } while (0)
int params_validate(const aeb_params_t *p)
{
    int bad = 0;
    CHECK(p->radar_min_range_m > 0 && p->radar_min_range_m < p->radar_max_range_m, "radar range limits");
    CHECK(p->radar_timeout_ms >= 20, "radar_timeout_ms >= 20");
    CHECK(p->track_confirm_hits >= 1 && p->track_max_misses >= 0, "track hits/misses");
    CHECK(p->imu_forward_axis >= 0 && p->imu_forward_axis <= 2, "imu_forward_axis 0..2");
    CHECK(p->imu_forward_sign == 1 || p->imu_forward_sign == -1, "imu_forward_sign +-1");
    CHECK(p->imu_lpf_alpha > 0.0f && p->imu_lpf_alpha <= 1.0f, "imu_lpf_alpha (0,1]");
    CHECK(p->impact_threshold_g > 1.0f && p->impact_threshold_g < 8.0f,
          "impact threshold must be within +-8 g range");
    CHECK(p->ttc_critical_s > 0 && p->ttc_critical_s < p->ttc_strong_s &&
              p->ttc_strong_s < p->ttc_partial_s && p->ttc_partial_s < p->ttc_warning_s,
          "TTC thresholds must satisfy critical < strong < partial < warning");
    for (int i = 0; i < 3; i++) CHECK(params_decel(p, i) > 0.05f, "decelerations must be > 0");
    CHECK(params_decel(p, 0) <= params_decel(p, 1) && params_decel(p, 1) <= params_decel(p, 2),
          "decel partial <= strong <= full");
    CHECK(p->safety_factor >= 1.0f, "safety_factor >= 1.0");
    CHECK(p->t_delay_s >= 0.0f && p->t_delay_s < 1.0f, "t_delay_s 0..1");
    CHECK(p->hyst_factor >= 1.0f, "hyst_factor >= 1");
    CHECK(p->partial_margin_k >= 1.0f && p->warning_margin_k >= p->partial_margin_k, "margin factors");
    CHECK(p->corridor_half_width_m > 0.0f, "corridor width");
    CHECK(p->cruise_duty >= 0 && p->cruise_duty <= p->motor_max_duty, "cruise_duty <= motor_max_duty");
    CHECK(p->motor_max_duty > 0 && p->motor_max_duty <= 100, "motor_max_duty 0..100");
    CHECK(p->motor_axles >= 1 && p->motor_axles <= 3, "motor_axles 1..3");
    CHECK(p->strong_duty <= p->partial_duty && p->partial_duty <= p->cruise_duty,
          "strong <= partial <= cruise duty");
    CHECK(p->pwm_hz > 0 && p->pwm_hz <= 25000, "pwm_hz (BTS7960 max ~25 kHz)");
    CHECK(p->pwm_hz_fallback > 0 && p->pwm_hz_fallback <= 1000, "pwm_hz_fallback");
    CHECK(p->watchdog_timeout_ms >= 30, "watchdog timeout");
    CHECK(p->braking_deadline_us >= 5000, "braking deadline");
    for (int pin = 0; pin < 6; pin++) {
        const int pins[6] = {p->pin_front_rpwm, p->pin_front_lpwm, p->pin_front_en,
                             p->pin_rear_rpwm,  p->pin_rear_lpwm,  p->pin_rear_en};
        CHECK(pins[pin] >= 4 && pins[pin] <= 27 && pins[pin] != 14 && pins[pin] != 15,
              "motor GPIO must be 4..27 and not 14/15 (UART) or 2/3 (I2C)");
        for (int j = pin + 1; j < 6; j++) CHECK(pins[pin] != pins[j], "duplicate motor GPIO");
    }
    return bad;
}
