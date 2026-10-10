#include "aeb_controller.h"
#include "aeb_cycle.h"
#include "brake.h"
#include "motor_bts7960.h"
#include "watchdog.h"
#include "ipc.h"
#include "shared_state.h"
#include "platform_qnx.h"
#include "logger.h"
#include "params.h"
#include "config.h"
#include <errno.h>
#include <string.h>
#include <sys/neutrino.h>
atomic_int g_inject_hang_ms;
static bool is_safe_entry(aeb_state_t s)
{
    return s == AEB_EMERGENCY_BRAKE || s == AEB_RADAR_FAULT || s == AEB_IMU_FAULT || s == AEB_SYSTEM_FAULT;
}
void *prediction_thread(void *arg)
{
    (void)arg;
    int chid, coid;
    timer_t tid;
    if (plat_channel_create(&chid, &coid) != 0 ||
        plat_timer_periodic(coid, PC_TIMER, PRIO_PREDICTION, PREDICT_PERIOD_MS * 1000u, &tid) != 0) {
        log_event(SEV_CRIT, "prediction thread: channel/timer failed (%s)", strerror(errno));
        fault_set(FAULT_WDOG_THREAD);
        motor_force_safe(true, "prediction thread start failure");
        return NULL;
    }
    const int bcoid = brake_coid();
    aeb_sm_t sm;
    aeb_sm_init(&sm, plat_now_ns());
    uint32_t seq = 0, last_live = 0;
    float last_lat_us = 0.0f;
    while (atomic_load(&g_running)) {
        struct _pulse p;
        if (MsgReceivePulse(chid, &p, sizeof p, NULL) == -1) continue;
        if (p.code != PC_TIMER) continue;
        hb_beat(HB_PREDICT);
        int hang = atomic_exchange(&g_inject_hang_ms, 0);
        if (hang > 0) {
            log_event(SEV_WARN, "[sim] prediction thread hangs for %d ms (fault injection)", hang);
            plat_sleep_ms((unsigned)hang);
        }
        radar_state_t rs;
        shared_radar_get(&rs);
        imu_state_t im;
        shared_imu_get(&im);
        vehicle_state_t vs;
        shared_veh_get(&vs);
        const uint64_t now = plat_now_ns();
        aeb_cycle_in_t in;
        in.now_ns = now;
        in.radar = &rs;
        in.imu = &im;
        in.veh = &vs;
        in.latched_faults = fault_get();
        in.cmd_go = atomic_exchange(&g_cmd_go, false);
        in.cmd_stop = atomic_exchange(&g_cmd_stop, false);
        aeb_cycle_out_t out;
        aeb_cycle(&sm, &g_params, &in, &out);
        out.status.cycle = ++seq;
        shared_aeb_put(&out.status);
        if (out.live_faults != last_live) {
            uint32_t newb = out.live_faults & ~last_live;
            if (newb & FAULT_RADAR_TIMEOUT) {
                if (rs.t_ns)
                    log_event(SEV_ERROR, "radar not fresh (age %.0f ms)", (double)age_ns(now, rs.t_ns) / 1e6);
                else
                    log_event(SEV_INFO, "waiting for first valid radar frame");
            }
            if (newb & FAULT_IMU_TIMEOUT) log_event(SEV_WARN, "IMU not available/fresh");
            last_live = out.live_faults;
        }
        if (out.state_changed) {
            const aeb_status_t *s = &out.status;
            log_event(s->state >= AEB_EMERGENCY_BRAKE ? SEV_WARN : SEV_INFO,
                      "STATE %s -> %s | risk=%s r=%.2fm vc=%.2fm/s ttc=%.2fs dsafe P/S/F=%.2f/%.2f/%.2f%s",
                      aeb_state_name(out.prev_state), aeb_state_name(s->state), risk_name(s->risk),
                      s->threat.valid ? s->threat.range_m : -1.0f, s->threat.v_close_mps,
                      s->ttc_valid ? s->ttc_s : -1.0f, s->d_safe_m[0] > 99 ? 99.0f : s->d_safe_m[0],
                      s->d_safe_m[1] > 99 ? 99.0f : s->d_safe_m[1],
                      s->d_safe_m[2] > 99 ? 99.0f : s->d_safe_m[2], s->unavoidable ? " UNAVOIDABLE" : "");
        }
        float v_ref = out.status.threat.valid ? out.status.threat.v_close_mps
                                              : (vs.ego_valid ? vs.ego_speed_mps : 0.0f);
        if (out.state_changed && is_safe_entry(out.status.state)) {
            atomic_store(&g_emerg_t_detect_ns, now);
            atomic_store(&g_emerg_seq_sent, seq);
            watchdog_arm_deadline(seq);
            if (MsgSendPulse(bcoid, PRIO_EMERGENCY_PULSE, PC_EMERGENCY, EMERG_VALUE(seq, out.status.state)) ==
                -1) {
                log_event(SEV_CRIT, "emergency pulse failed (%s) -> direct safe", strerror(errno));
                motor_force_safe(true, "emergency pulse failed");
            }
        } else {
            brake_decision_msg_t m;
            memset(&m, 0, sizeof m);
            m.type = MSG_BRAKE_DECISION;
            m.seq = seq;
            m.t_decision_ns = now;
            m.t_sensor_ns = rs.t_ns;
            m.state = out.status.state;
            m.cmd = out.cmd;
            m.v_ref_mps = v_ref;
            m.imu_ok = out.status.imu_ok;
            brake_reply_t rep;
            uint64_t to_ns = (uint64_t)g_params.braking_deadline_us * 1000ull;
            TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY, NULL, &to_ns, NULL);
            if (MsgSend(bcoid, &m, sizeof m, &rep, sizeof rep) == -1) {
                if (!atomic_load(&g_running)) break;
                log_event(SEV_CRIT, "MsgSend to brake failed (%s) -> direct safe", strerror(errno));
                fault_set(FAULT_DEADLINE_MISS);
                motor_force_safe(true, "brake controller unreachable");
            } else {
                last_lat_us = (float)((double)(rep.t_applied_ns - now) / 1000.0);
                if (rep.status != 0) log_event(SEV_ERROR, "brake controller reported GPIO error");
            }
        }
        log_cycle(&out.status, out.cmd, last_lat_us);
    }
    timer_delete(tid);
    plat_channel_destroy(chid, coid);
    return NULL;
}
