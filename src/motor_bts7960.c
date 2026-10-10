#include "motor_bts7960.h"
#include "gpio_rpi.h"
#include "params.h"
#include "config.h"
#include "shared_state.h"
#include "platform_qnx.h"
#include "logger.h"
#include <errno.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
static motor_backend_t g_backend = MOTOR_BACKEND_NONE;
static atomic_bool g_latched;
static atomic_int g_applied_type = MOTOR_DISABLE;
static _Atomic float g_applied_duty;
static _Atomic uint64_t g_applied_ns;
static _Atomic float g_soft_duty;
static atomic_int g_soft_drv_mask = DRV_BOTH;
static int pin_rpwm(int side)
{
    return side == DRV_FRONT ? g_params.pin_front_rpwm : g_params.pin_rear_rpwm;
}
static int pin_lpwm(int side)
{
    return side == DRV_FRONT ? g_params.pin_front_lpwm : g_params.pin_rear_lpwm;
}
static int pin_en(int side)
{
    return side == DRV_FRONT ? g_params.pin_front_en : g_params.pin_rear_en;
}
static int active_mask(void)
{
    int a = g_params.motor_axles & DRV_BOTH;
    return a ? a : DRV_BOTH;
}
const char *motor_backend_name(motor_backend_t b)
{
    switch (b) {
    case MOTOR_BACKEND_NONE: return "DRY-RUN (no GPIO)";
    case MOTOR_BACKEND_HWPWM: return "HW-PWM (rpi_gpio)";
    case MOTOR_BACKEND_SOFTPWM: return "SOFT-PWM thread";
    default: return "?";
    }
}
motor_backend_t motor_backend(void)
{
    return g_backend;
}
bool motor_safe_latched(void)
{
    return atomic_load(&g_latched);
}
static int w(int pin, int level)
{
    if (g_backend == MOTOR_BACKEND_NONE) return 0;
    return gpio_write(pin, level) == 0 ? 0 : 1;
}
static int set_duty(int side, float duty)
{
    if (g_backend == MOTOR_BACKEND_NONE) return 0;
    if (g_backend == MOTOR_BACKEND_HWPWM) return gpio_pwm_duty(pin_rpwm(side), duty) == 0 ? 0 : 1;
    if (duty <= 0.0f) return w(pin_rpwm(side), 0);
    return 0;
}
static int sides_disable(int mask)
{
    int e = 0;
    for (int s = DRV_FRONT; s <= DRV_REAR; s++) {
        if (!(mask & s)) continue;
        e += set_duty(s, 0.0f);
        e += w(pin_lpwm(s), 0);
        e += w(pin_en(s), 0);
    }
    return e;
}
static int sides_brake(int mask)
{
    int e = 0;
    if (mask & ~active_mask()) e += sides_disable(mask & ~active_mask());
    mask &= active_mask();
    for (int s = DRV_FRONT; s <= DRV_REAR; s++) {
        if (!(mask & s)) continue;
        e += set_duty(s, 0.0f);
        e += w(pin_lpwm(s), 0);
    }
    if (e == 0) {
        for (int s = DRV_FRONT; s <= DRV_REAR; s++)
            if (mask & s) e += w(pin_en(s), 1);
    } else {
        sides_disable(mask);
    }
    return e;
}
static int sides_drive(int mask, float duty)
{
    int e = 0;
    if (mask & ~active_mask()) e += sides_disable(mask & ~active_mask());
    mask &= active_mask();
    for (int s = DRV_FRONT; s <= DRV_REAR; s++) {
        if (!(mask & s)) continue;
        e += w(pin_lpwm(s), 0);
        e += w(pin_en(s), 1);
        e += set_duty(s, duty);
    }
    return e;
}
static void record(motor_cmd_type_t t, float d)
{
    atomic_store(&g_applied_type, (int)t);
    atomic_store(&g_applied_duty, d);
    atomic_store(&g_applied_ns, plat_now_ns());
}
int motor_init(motor_backend_t requested)
{
    g_backend = requested;
    atomic_store(&g_soft_duty, 0.0f);
    if (requested == MOTOR_BACKEND_NONE) {
        record(MOTOR_DISABLE, 0);
        return 0;
    }
    if (gpio_open(GPIO_MSG_PATH) != 0) {
        fprintf(stderr, "[motor] cannot open %s (%s) - is rpi_gpio running?\n", GPIO_MSG_PATH,
                strerror(errno));
        g_backend = MOTOR_BACKEND_NONE;
        return -1;
    }
    const int pins[6] = {g_params.pin_front_rpwm, g_params.pin_front_lpwm, g_params.pin_front_en,
                         g_params.pin_rear_rpwm,  g_params.pin_rear_lpwm,  g_params.pin_rear_en};
    for (int i = 0; i < 6; i++) {
        if (gpio_set_output(pins[i]) != 0 || gpio_write(pins[i], 0) != 0) {
            fprintf(stderr, "[motor] GPIO%d init failed (%s)\n", pins[i], strerror(errno));
            return -1;
        }
        gpio_set_pull(pins[i], 1);
    }
    if (requested == MOTOR_BACKEND_HWPWM) {
        if (gpio_pwm_setup(g_params.pin_front_rpwm, (unsigned)g_params.pwm_hz) != 0 ||
            gpio_pwm_setup(g_params.pin_rear_rpwm, (unsigned)g_params.pwm_hz) != 0 ||
            gpio_pwm_duty(g_params.pin_front_rpwm, 0.0f) != 0 ||
            gpio_pwm_duty(g_params.pin_rear_rpwm, 0.0f) != 0) {
            fprintf(stderr, "[motor] hardware PWM setup rejected (%s) -> software PWM at %d Hz\n",
                    strerror(errno), g_params.pwm_hz_fallback);
            g_backend = MOTOR_BACKEND_SOFTPWM;
            gpio_set_output(g_params.pin_front_rpwm);
            gpio_write(g_params.pin_front_rpwm, 0);
            gpio_set_output(g_params.pin_rear_rpwm);
            gpio_write(g_params.pin_rear_rpwm, 0);
        }
    }
    record(MOTOR_DISABLE, 0);
    return 0;
}
int motor_apply(motor_cmd_t c)
{
    if (c.type == MOTOR_DRIVE) {
        if (c.duty_pct < 0) c.duty_pct = 0;
        if (c.duty_pct > g_params.motor_max_duty) c.duty_pct = g_params.motor_max_duty;
    }
    if (atomic_load(&g_latched) && c.type == MOTOR_DRIVE) c.type = MOTOR_BRAKE;
    int e;
    atomic_store(&g_soft_drv_mask, DRV_BOTH);
    switch (c.type) {
    case MOTOR_DRIVE:
        atomic_store(&g_soft_duty, c.duty_pct);
        e = sides_drive(DRV_BOTH, c.duty_pct);
        break;
    case MOTOR_BRAKE:
        atomic_store(&g_soft_duty, 0.0f);
        e = sides_brake(DRV_BOTH);
        break;
    case MOTOR_DISABLE:
    default:
        atomic_store(&g_soft_duty, 0.0f);
        e = sides_disable(DRV_BOTH);
        c.type = MOTOR_DISABLE;
        break;
    }
    record(c.type, c.type == MOTOR_DRIVE ? c.duty_pct : 0.0f);
    if (c.type == MOTOR_DRIVE && atomic_load(&g_latched)) {
        atomic_store(&g_soft_duty, 0.0f);
        e += sides_brake(DRV_BOTH);
        record(MOTOR_BRAKE, 0);
    }
    if (e) {
        fault_set(FAULT_MOTOR_GPIO);
        log_event(SEV_CRIT, "MOTOR GPIO error (%d failed writes) while applying %s", e,
                  motor_cmd_name(c.type));
        return -1;
    }
    return 0;
}
int motor_test_driver(int mask, float duty)
{
    if (atomic_load(&g_latched)) return -1;
    int other = DRV_BOTH & ~mask;
    int e = sides_disable(other);
    atomic_store(&g_soft_drv_mask, mask);
    if (duty > 0.0f) {
        atomic_store(&g_soft_duty, duty);
        e += sides_drive(mask, duty);
        record(MOTOR_DRIVE, duty);
    } else {
        atomic_store(&g_soft_duty, 0.0f);
        e += sides_brake(mask);
        record(MOTOR_BRAKE, 0);
    }
    return e ? -1 : 0;
}
void motor_force_safe(bool brake, const char *why)
{
    bool was = atomic_exchange(&g_latched, true);
    atomic_store(&g_soft_duty, 0.0f);
    int e = brake ? sides_brake(DRV_BOTH) : sides_disable(DRV_BOTH);
    record(brake && e == 0 ? MOTOR_BRAKE : MOTOR_DISABLE, 0);
    if (!was)
        log_event(SEV_CRIT, "MOTOR SAFE-LATCH set (%s) -> %s%s", why ? why : "?", brake ? "BRAKE" : "DISABLE",
                  e ? " [GPIO ERRORS]" : "");
}
motor_cmd_t motor_get_applied(void)
{
    motor_cmd_t c = {(motor_cmd_type_t)atomic_load(&g_applied_type), atomic_load(&g_applied_duty)};
    return c;
}
uint64_t motor_last_apply_ns(void)
{
    return atomic_load(&g_applied_ns);
}
void motor_shutdown(void)
{
    atomic_store(&g_latched, true);
    atomic_store(&g_soft_duty, 0.0f);
    sides_disable(DRV_BOTH);
    record(MOTOR_DISABLE, 0);
}
static void soft_pwm_loop(void)
{
    const uint64_t period =
        1000000000ull / (uint64_t)(g_params.pwm_hz_fallback > 0 ? g_params.pwm_hz_fallback : 100);
    uint64_t t = plat_now_ns();
    int level = -1;
    while (atomic_load(&g_running)) {
        hb_beat(HB_MOTOR);
        float d = atomic_load(&g_soft_duty);
        int mask = atomic_load(&g_soft_drv_mask) & active_mask();
        if (atomic_load(&g_latched)) d = 0.0f;
        if (d <= 0.5f || d >= 99.5f) {
            int want = d >= 99.5f ? 1 : 0;
            if (want != level) {
                if (mask & DRV_FRONT) w(g_params.pin_front_rpwm, want);
                if (mask & DRV_REAR) w(g_params.pin_rear_rpwm, want);
                level = want;
            }
            t += period;
            plat_sleep_until_ns(t);
        } else {
            if (mask & DRV_FRONT) w(g_params.pin_front_rpwm, 1);
            if (mask & DRV_REAR) w(g_params.pin_rear_rpwm, 1);
            plat_sleep_until_ns(t + (uint64_t)((double)period * d / 100.0));
            w(g_params.pin_front_rpwm, 0);
            w(g_params.pin_rear_rpwm, 0);
            level = 0;
            t += period;
            plat_sleep_until_ns(t);
        }
        uint64_t now = plat_now_ns();
        if (now > t + 5 * period) t = now;
    }
    w(g_params.pin_front_rpwm, 0);
    w(g_params.pin_rear_rpwm, 0);
}
static void readback_loop(void)
{
    int mismatch = 0;
    bool readback_ok = true;
    uint64_t next = plat_now_ns();
    uint64_t last_safe_refresh = 0;
    while (atomic_load(&g_running)) {
        hb_beat(HB_MOTOR);
        uint64_t now = plat_now_ns();
        if (g_backend != MOTOR_BACKEND_NONE && readback_ok) {
            motor_cmd_t c = motor_get_applied();
            int exp_en = (c.type == MOTOR_DISABLE) ? 0 : 1;
            int exp_en_f = (active_mask() & DRV_FRONT) ? exp_en : 0;
            int exp_en_r = (active_mask() & DRV_REAR) ? exp_en : 0;
            int en_l = -1, en_r = -1, lp_l = -1, lp_r = -1;
            if (gpio_read(g_params.pin_front_en, &en_l) != 0 || gpio_read(g_params.pin_rear_en, &en_r) != 0 ||
                gpio_read(g_params.pin_front_lpwm, &lp_l) != 0 ||
                gpio_read(g_params.pin_rear_lpwm, &lp_r) != 0) {
                readback_ok = false;
                log_event(SEV_WARN, "GPIO read-back not supported (%s) - check disabled", strerror(errno));
            } else if (age_ns(plat_now_ns(), motor_last_apply_ns()) > 5000000ull) {
                if (en_l != exp_en_f || en_r != exp_en_r || lp_l != 0 || lp_r != 0) {
                    if (++mismatch == 3) {
                        fault_set(FAULT_MOTOR_GPIO);
                        log_event(SEV_CRIT,
                                  "MOTOR control-signal mismatch: EN F/R=%d/%d (exp %d/%d) LPWM F/R=%d/%d",
                                  en_l, en_r, exp_en_f, exp_en_r, lp_l, lp_r);
                    }
                } else {
                    mismatch = 0;
                }
            }
        }
        if (atomic_load(&g_latched) && now - last_safe_refresh > 100000000ull) {
            motor_cmd_t c = motor_get_applied();
            if (c.type == MOTOR_BRAKE)
                sides_brake(DRV_BOTH);
            else
                sides_disable(DRV_BOTH);
            last_safe_refresh = now;
        }
        next += (uint64_t)MOTOR_READBACK_MS * 1000000ull;
        if (next < now) next = now;
        plat_sleep_until_ns(next);
    }
}
void *motor_thread(void *arg)
{
    (void)arg;
    if (g_backend == MOTOR_BACKEND_SOFTPWM)
        soft_pwm_loop();
    else
        readback_loop();
    return NULL;
}
int motor_park_pins(void)
{
    if (gpio_open(GPIO_MSG_PATH) != 0) return -1;
    const int pins[6] = {g_params.pin_front_en,  g_params.pin_rear_en,    g_params.pin_front_rpwm,
                         g_params.pin_rear_rpwm, g_params.pin_front_lpwm, g_params.pin_rear_lpwm};
    int e = 0;
    for (int i = 0; i < 6; i++) {
        if (gpio_set_output(pins[i]) != 0 || gpio_write(pins[i], 0) != 0) e++;
    }
    g_backend = MOTOR_BACKEND_NONE;
    record(MOTOR_DISABLE, 0);
    return e ? -1 : 0;
}
void motor_signal_safe(void)
{
    atomic_store(&g_latched, true);
    atomic_store(&g_soft_duty, 0.0f);
    if (!gpio_is_open()) return;
    gpio_write(g_params.pin_front_en, 0);
    gpio_write(g_params.pin_rear_en, 0);
    if (g_backend == MOTOR_BACKEND_HWPWM) {
        gpio_pwm_duty(g_params.pin_front_rpwm, 0.0f);
        gpio_pwm_duty(g_params.pin_rear_rpwm, 0.0f);
    } else {
        gpio_write(g_params.pin_front_rpwm, 0);
        gpio_write(g_params.pin_rear_rpwm, 0);
    }
}
