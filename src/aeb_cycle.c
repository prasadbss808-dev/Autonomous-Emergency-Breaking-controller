#include "aeb_cycle.h"
#include <string.h>
bool radar_is_fresh(const radar_state_t *r, uint64_t now_ns, const aeb_params_t *p)
{
    if (!r || !r->link_ok || r->t_ns == 0) return false;
    return age_ns(now_ns, r->t_ns) <= (uint64_t)p->radar_timeout_ms * 1000000ull;
}
bool imu_is_fresh(const imu_state_t *m, uint64_t now_ns, const aeb_params_t *p)
{
    if (!m || !m->valid || !m->calibrated || m->t_ns == 0) return false;
    return age_ns(now_ns, m->t_ns) <= (uint64_t)p->imu_timeout_ms * 1000000ull;
}
void aeb_cycle(aeb_sm_t *sm, const aeb_params_t *p, const aeb_cycle_in_t *in, aeb_cycle_out_t *out)
{
    memset(out, 0, sizeof(*out));
    const uint64_t now = in->now_ns;
    const bool radar_ok = radar_is_fresh(in->radar, now, p);
    const bool imu_ok = imu_is_fresh(in->imu, now, p);
    const bool motor_ok = (in->latched_faults & FAULTS_MOTOR) == 0;
    const bool system_ok = (in->latched_faults & FAULTS_SYSTEM) == 0;
    const bool impact = (in->imu && in->imu->impact) || (in->latched_faults & FAULT_IMPACT);
    const bool degraded = !imu_ok && p->imu_fault_policy == 1;
    if (!radar_ok) out->live_faults |= FAULT_RADAR_TIMEOUT;
    if (!imu_ok) out->live_faults |= FAULT_IMU_TIMEOUT;
    threat_t th;
    memset(&th, 0, sizeof th);
    if (radar_ok) radar_select_threat(in->radar->tracks, in->radar->n_tracks, p, &th);
    risk_level_t lvl_enter = aeb_classify(&th, p, 1.0f, &out->eval);
    risk_level_t lvl_exit = aeb_classify(&th, p, p->hyst_factor, NULL);
    aeb_sm_input_t si;
    memset(&si, 0, sizeof si);
    si.now_ns = now;
    si.radar_ok = radar_ok;
    si.imu_ok = imu_ok;
    si.motor_ok = motor_ok;
    si.system_ok = system_ok;
    si.impact = impact;
    si.level_enter = lvl_enter;
    si.level_exit = lvl_exit;
    si.vehicle_stopped = in->veh ? in->veh->stopped : false;
    si.cmd_go = in->cmd_go;
    si.cmd_stop = in->cmd_stop;
    out->prev_state = sm->state;
    aeb_state_t st = aeb_sm_step(sm, &si, p);
    out->state_changed = (st != out->prev_state);
    out->cmd = aeb_motor_cmd_for_state(st, sm->has_driven, degraded, p);
    aeb_status_t *s = &out->status;
    s->t_ns = now;
    s->state = st;
    s->risk = lvl_enter;
    s->threat = th;
    s->ttc_valid = out->eval.ttc_valid;
    s->ttc_s = out->eval.ttc_s;
    for (int i = 0; i < 3; i++) s->d_safe_m[i] = out->eval.d_safe_m[i];
    s->unavoidable = out->eval.unavoidable;
    s->radar_ok = radar_ok;
    s->imu_ok = imu_ok;
    s->motor_ok = motor_ok;
    s->system_ok = system_ok;
    s->degraded = degraded;
    s->faults = (in->latched_faults | out->live_faults) & ~FAULTS_INFO;
    if (in->veh) {
        s->ego_valid = in->veh->ego_valid;
        s->ego_speed_mps = in->veh->ego_speed_mps;
    }
    s->a_long = (in->imu && imu_ok) ? in->imu->a_long : 0.0f;
}
