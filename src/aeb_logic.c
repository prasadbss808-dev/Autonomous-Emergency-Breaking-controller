#include "aeb_logic.h"
#include <string.h>
#define BIG_DISTANCE_M 1.0e9f
#define BIG_TTC_S 1.0e9f
float aeb_sqrtf(float x)
{
    if (!(x > 0.0f)) return 0.0f;
    if (x > 3.0e38f) return 1.8e19f;
    union {
        float f;
        uint32_t i;
    } u;
    u.f = x;
    u.i = 0x1fbd1df5u + (u.i >> 1);
    float y = u.f;
    y = 0.5f * (y + x / y);
    y = 0.5f * (y + x / y);
    y = 0.5f * (y + x / y);
    return y;
}
bool aeb_ttc(float range_m, float v_close_mps, float v_min_mps, float *ttc_s)
{
    if (!(range_m > 0.0f) || !(v_close_mps > v_min_mps) || v_close_mps <= 0.0f) {
        *ttc_s = BIG_TTC_S;
        return false;
    }
    *ttc_s = range_m / v_close_mps;
    return true;
}
float aeb_stopping_distance(float v_mps, float t_delay_s, float decel_mps2)
{
    if (!(v_mps > 0.0f)) return 0.0f;
    if (!(decel_mps2 > 0.0f)) return BIG_DISTANCE_M;
    return v_mps * t_delay_s + (v_mps * v_mps) / (2.0f * decel_mps2);
}
float aeb_safe_distance(float v_mps, float t_delay_s, float decel_mps2, float sf)
{
    float d = aeb_stopping_distance(v_mps, t_delay_s, decel_mps2);
    if (d >= BIG_DISTANCE_M) return BIG_DISTANCE_M;
    return sf * d;
}
static risk_level_t max_lvl(risk_level_t a, risk_level_t b)
{
    return a > b ? a : b;
}
risk_level_t aeb_classify(const threat_t *th, const aeb_params_t *p, float scale, risk_eval_t *ev)
{
    risk_eval_t local;
    if (!ev) ev = &local;
    memset(ev, 0, sizeof(*ev));
    ev->ttc_s = BIG_TTC_S;
    if (!th || !th->valid) {
        ev->level = RISK_NONE;
        return RISK_NONE;
    }
    const float r = th->range_m;
    const float v = th->v_close_mps;
    ev->ttc_valid = aeb_ttc(r, v, p->v_close_min_mps, &ev->ttc_s);
    risk_level_t lt = RISK_NONE;
    if (ev->ttc_valid) {
        const float t = ev->ttc_s;
        if (t <= p->ttc_critical_s * scale)
            lt = RISK_FULL;
        else if (t <= p->ttc_strong_s * scale)
            lt = RISK_STRONG;
        else if (t <= p->ttc_partial_s * scale)
            lt = RISK_PARTIAL;
        else if (t <= p->ttc_warning_s * scale)
            lt = RISK_WARNING;
    }
    const float v_eff = (v > p->v_close_min_mps) ? v : 0.0f;
    for (int i = 0; i < 3; i++)
        ev->d_safe_m[i] = aeb_safe_distance(v_eff, p->t_delay_s, params_decel(p, i), p->safety_factor);
    risk_level_t ld = RISK_NONE;
    if (v_eff > 0.0f) {
        const float dp = ev->d_safe_m[LVL_IDX_PARTIAL];
        const float ds = ev->d_safe_m[LVL_IDX_STRONG];
        if (r <= ds * scale)
            ld = RISK_FULL;
        else if (r <= dp * scale)
            ld = RISK_STRONG;
        else if (r <= dp * p->partial_margin_k * scale)
            ld = RISK_PARTIAL;
        else if (r <= dp * p->warning_margin_k * scale)
            ld = RISK_WARNING;
        ev->unavoidable = (r <= ev->d_safe_m[LVL_IDX_FULL]);
    }
    risk_level_t lp = RISK_NONE;
    if (p->proximity_stop_m > 0.0f && r <= p->proximity_stop_m * scale) lp = RISK_FULL;
    ev->level_ttc = lt;
    ev->level_dist = ld;
    ev->level = max_lvl(max_lvl(lt, ld), lp);
    return ev->level;
}
static aeb_state_t state_for_level(risk_level_t l)
{
    switch (l) {
    case RISK_WARNING: return AEB_WARNING;
    case RISK_PARTIAL: return AEB_PARTIAL_BRAKE;
    case RISK_STRONG: return AEB_STRONG_BRAKE;
    case RISK_FULL: return AEB_EMERGENCY_BRAKE;
    case RISK_NONE:
    default: return AEB_NORMAL;
    }
}
static void sm_enter(aeb_sm_t *sm, aeb_state_t s, uint64_t now)
{
    if (sm->state != s) {
        sm->state = s;
        sm->t_enter_ns = now;
        sm->transitions++;
    }
    sm->downgrade_pending = false;
}
void aeb_sm_init(aeb_sm_t *sm, uint64_t now_ns)
{
    memset(sm, 0, sizeof(*sm));
    sm->state = AEB_STOPPED;
    sm->t_enter_ns = now_ns;
    sm->has_driven = false;
}
static bool is_active(aeb_state_t s)
{
    return s <= AEB_EMERGENCY_BRAKE;
}
aeb_state_t aeb_sm_step(aeb_sm_t *sm, const aeb_sm_input_t *in, const aeb_params_t *p)
{
    const uint64_t now = in->now_ns;
    const aeb_state_t s = sm->state;
    const uint64_t ms_in_state = (now >= sm->t_enter_ns) ? (now - sm->t_enter_ns) / 1000000ull : 0;
    const bool sensors_ok = in->radar_ok && (in->imu_ok || p->imu_fault_policy == 1);
    if (!in->system_ok) {
        sm_enter(sm, AEB_SYSTEM_FAULT, now);
        return sm->state;
    }
    if (!in->motor_ok) {
        sm_enter(sm, AEB_MOTOR_FAULT, now);
        return sm->state;
    }
    if (s == AEB_SYSTEM_FAULT || s == AEB_MOTOR_FAULT) return s;
    if (in->impact && is_active(s) && s != AEB_EMERGENCY_BRAKE) {
        sm_enter(sm, AEB_EMERGENCY_BRAKE, now);
        return sm->state;
    }
    if (is_active(s)) {
        if (!in->radar_ok) {
            sm_enter(sm, AEB_RADAR_FAULT, now);
            return sm->state;
        }
        if (!in->imu_ok && p->imu_fault_policy == 0) {
            sm_enter(sm, AEB_IMU_FAULT, now);
            return sm->state;
        }
    }
    if (in->cmd_stop) {
        sm_enter(sm, AEB_STOPPED, now);
        return sm->state;
    }
    switch (s) {
    case AEB_RADAR_FAULT:
    case AEB_IMU_FAULT:
        if (sensors_ok && in->cmd_go && !in->impact && in->level_enter < RISK_PARTIAL) {
            sm->has_driven = true;
            sm_enter(sm, AEB_NORMAL, now);
        }
        break;
    case AEB_STOPPED: {
        const bool clear = in->level_enter < RISK_PARTIAL;
        const bool auto_ok = (p->auto_resume && sm->has_driven && ms_in_state >= 2000) ||
                             (p->auto_start && !sm->has_driven && ms_in_state >= 1500);
        if (sensors_ok && clear && !in->impact && (in->cmd_go || auto_ok)) {
            sm->has_driven = true;
            sm_enter(sm, AEB_NORMAL, now);
        }
        break;
    }
    case AEB_EMERGENCY_BRAKE:
        if (ms_in_state >= (uint64_t)p->emergency_min_hold_ms &&
            (in->vehicle_stopped || ms_in_state >= (uint64_t)p->emergency_max_hold_ms))
            sm_enter(sm, AEB_STOPPED, now);
        break;
    case AEB_NORMAL:
    case AEB_WARNING:
    case AEB_PARTIAL_BRAKE:
    case AEB_STRONG_BRAKE: {
        const aeb_state_t target = state_for_level(in->level_enter);
        if (target > s) {
            sm_enter(sm, target, now);
            break;
        }
        if (s == AEB_STRONG_BRAKE && in->vehicle_stopped && in->level_exit >= RISK_PARTIAL) {
            sm_enter(sm, AEB_STOPPED, now);
            break;
        }
        const aeb_state_t exit_state = state_for_level(in->level_exit);
        if (exit_state < s) {
            if (!sm->downgrade_pending) {
                sm->downgrade_pending = true;
                sm->t_downgrade_ns = now;
            } else if ((now - sm->t_downgrade_ns) / 1000000ull >= (uint64_t)p->downgrade_hold_ms) {
                sm_enter(sm, (aeb_state_t)(s - 1), now);
            }
        } else {
            sm->downgrade_pending = false;
        }
        break;
    }
    default: sm_enter(sm, AEB_SYSTEM_FAULT, now); break;
    }
    return sm->state;
}
int aeb_state_brake_idx(aeb_state_t s)
{
    switch (s) {
    case AEB_PARTIAL_BRAKE: return LVL_IDX_PARTIAL;
    case AEB_STRONG_BRAKE: return LVL_IDX_STRONG;
    case AEB_EMERGENCY_BRAKE:
    case AEB_RADAR_FAULT:
    case AEB_IMU_FAULT:
    case AEB_SYSTEM_FAULT: return LVL_IDX_FULL;
    default: return -1;
    }
}
static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}
motor_cmd_t aeb_motor_cmd_for_state(aeb_state_t s, bool has_driven, bool degraded, const aeb_params_t *p)
{
    motor_cmd_t c = {MOTOR_BRAKE, 0.0f};
    switch (s) {
    case AEB_NORMAL:
        c.type = MOTOR_DRIVE;
        c.duty_pct = p->cruise_duty;
        break;
    case AEB_WARNING:
        c.type = MOTOR_DRIVE;
        c.duty_pct = p->warning_duty;
        break;
    case AEB_PARTIAL_BRAKE:
        c.type = MOTOR_DRIVE;
        c.duty_pct = p->partial_duty;
        break;
    case AEB_STRONG_BRAKE:
        if (p->strong_duty > 0.0f) {
            c.type = MOTOR_DRIVE;
            c.duty_pct = p->strong_duty;
        } else {
            c.type = MOTOR_BRAKE;
        }
        break;
    case AEB_EMERGENCY_BRAKE:
    case AEB_RADAR_FAULT:
    case AEB_IMU_FAULT:
    case AEB_SYSTEM_FAULT: c.type = MOTOR_BRAKE; break;
    case AEB_STOPPED: c.type = has_driven ? MOTOR_BRAKE : MOTOR_DISABLE; break;
    case AEB_MOTOR_FAULT: c.type = MOTOR_DISABLE; break;
    default: c.type = MOTOR_DISABLE; break;
    }
    if (c.type == MOTOR_DRIVE) {
        float maxd = p->motor_max_duty;
        if (degraded && p->degraded_max_duty < maxd) maxd = p->degraded_max_duty;
        c.duty_pct = clampf(c.duty_pct, 0.0f, maxd);
    } else {
        c.duty_pct = 0.0f;
    }
    return c;
}
const char *aeb_state_name(aeb_state_t s)
{
    static const char *const n[AEB_STATE_COUNT] = {
        "NORMAL",  "WARNING",     "PARTIAL_BRAKE", "STRONG_BRAKE", "EMERGENCY_BRAKE",
        "STOPPED", "RADAR_FAULT", "IMU_FAULT",     "MOTOR_FAULT",  "SYSTEM_FAULT"};
    return ((unsigned)s < AEB_STATE_COUNT) ? n[s] : "?";
}
const char *risk_name(risk_level_t r)
{
    static const char *const n[] = {"NONE", "WARNING", "PARTIAL", "STRONG", "FULL"};
    return ((unsigned)r <= RISK_FULL) ? n[r] : "?";
}
const char *motor_cmd_name(motor_cmd_type_t t)
{
    switch (t) {
    case MOTOR_DISABLE: return "DISABLE";
    case MOTOR_DRIVE: return "DRIVE";
    case MOTOR_BRAKE: return "BRAKE";
    default: return "?";
    }
}
void brake_verify_start(brake_verify_t *bv, int lvl_idx, uint64_t now_ns, float v_ref_mps, bool imu_ok,
                        const aeb_params_t *p)
{
    memset(bv, 0, sizeof(*bv));
    bv->lvl_idx = lvl_idx;
    bv->t_cmd_ns = now_ns;
    bv->v_ref_mps = v_ref_mps;
    if (lvl_idx < 0 || lvl_idx > 2) {
        bv->status = VERIFY_IDLE;
        return;
    }
    bv->required = p->verify_min_decel[lvl_idx];
    if (!imu_ok || !(v_ref_mps >= p->verify_min_speed_mps)) {
        bv->status = VERIFY_SKIPPED;
        return;
    }
    bv->status = VERIFY_RUNNING;
}
int brake_verify_update(brake_verify_t *bv, uint64_t now_ns, float a_long, bool imu_ok, const aeb_params_t *p)
{
    if (bv->status != VERIFY_RUNNING) return bv->status;
    if (!imu_ok) {
        bv->status = VERIFY_SKIPPED;
        return bv->status;
    }
    uint64_t ms = (now_ns - bv->t_cmd_ns) / 1000000ull;
    if (ms < (uint64_t)p->verify_delay_ms) return bv->status;
    float decel = -a_long;
    if (decel > bv->peak_decel) bv->peak_decel = decel;
    if (bv->peak_decel >= bv->required)
        bv->status = VERIFY_PASS;
    else if (ms >= (uint64_t)p->verify_window_ms)
        bv->status = VERIFY_FAIL;
    return bv->status;
}
const char *verify_name(int st)
{
    switch (st) {
    case VERIFY_IDLE: return "IDLE";
    case VERIFY_RUNNING: return "RUNNING";
    case VERIFY_PASS: return "PASS";
    case VERIFY_FAIL: return "FAIL";
    case VERIFY_SKIPPED: return "SKIPPED";
    default: return "?";
    }
}
bool stop_detect_update(stop_det_t *sd, uint64_t now_ns, bool ego_valid, float ego_mps, bool imu_ok,
                        float a_long, motor_cmd_type_t cmd, const aeb_params_t *p)
{
    if (ego_valid) {
        sd->quiet = false;
        return ego_mps < p->v_stopped_mps;
    }
    if (!imu_ok || cmd == MOTOR_DRIVE) {
        sd->quiet = false;
        return false;
    }
    float a = a_long < 0 ? -a_long : a_long;
    if (a < p->stop_quiet_accel) {
        if (!sd->quiet) {
            sd->quiet = true;
            sd->t_quiet_ns = now_ns;
        }
        return (now_ns - sd->t_quiet_ns) / 1000000ull >= (uint64_t)p->stop_quiet_ms;
    }
    sd->quiet = false;
    return false;
}
void brake_sup_init(brake_sup_t *bs)
{
    memset(bs, 0, sizeof(*bs));
    bs->last_state = AEB_STOPPED;
}
bool brake_sup_on_state(brake_sup_t *bs, aeb_state_t s, uint64_t now_ns, float v_ref_mps, bool imu_ok,
                        const aeb_params_t *p)
{
    bool started = false;
    int idx = aeb_state_brake_idx(s);
    int last = aeb_state_brake_idx(bs->last_state);
    if (s != bs->last_state) {
        if (idx >= 0 && idx > last) {
            brake_verify_start(&bs->bv, idx, now_ns, v_ref_mps, imu_ok, p);
            started = true;
        } else if (idx < 0) {
            bs->bv.status = VERIFY_IDLE;
        }
        bs->last_state = s;
    }
    return started;
}
int brake_sup_tick(brake_sup_t *bs, uint64_t now_ns, float a_long, bool imu_ok, const aeb_params_t *p)
{
    return brake_verify_update(&bs->bv, now_ns, a_long, imu_ok, p);
}
