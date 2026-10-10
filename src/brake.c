#include "brake.h"
#include "ipc.h"
#include "aeb_logic.h"
#include "aeb_cycle.h"
#include "motor_bts7960.h"
#include "shared_state.h"
#include "platform_qnx.h"
#include "logger.h"
#include "params.h"
#include "config.h"
#include <errno.h>
#include <string.h>
#include <sys/neutrino.h>
static int g_chid = -1, g_coid = -1;
typedef union {
    struct _pulse pulse;
    uint16_t type;
    brake_decision_msg_t dec;
} brake_rx_t;
int brake_init(void)
{
    return plat_channel_create(&g_chid, &g_coid);
}
int brake_coid(void)
{
    return g_coid;
}
void brake_stop(void)
{
    if (g_coid >= 0) MsgSendPulse(g_coid, PRIO_BRAKE, PC_SHUTDOWN, 0);
}
static bool same_cmd(motor_cmd_t a, motor_cmd_t b)
{
    if (a.type != b.type) return false;
    if (a.type != MOTOR_DRIVE) return true;
    float d = a.duty_pct - b.duty_pct;
    return d < 0.05f && d > -0.05f;
}
void *brake_thread(void *arg)
{
    (void)arg;
    timer_t tid;
    if (plat_timer_periodic(g_coid, PC_TIMER, PRIO_BRAKE, BRAKE_TICK_MS * 1000u, &tid) != 0) {
        log_event(SEV_CRIT, "brake thread: timer creation failed (%s)", strerror(errno));
        motor_force_safe(true, "brake timer failure");
        fault_set(FAULT_WDOG_THREAD);
        return NULL;
    }
    brake_sup_t bs;
    brake_sup_init(&bs);
    motor_cmd_t last = motor_get_applied();
    aeb_state_t cur_state = AEB_STOPPED;
    bool verify_reported = true;
    for (;;) {
        brake_rx_t rx;
        struct _msg_info info;
        rcvid_t rcvid = MsgReceive(g_chid, &rx, sizeof rx, &info);
        if (rcvid == -1) {
            if (errno != EINTR) log_event(SEV_ERROR, "brake MsgReceive: %s", strerror(errno));
            continue;
        }
        uint64_t now = plat_now_ns();
        if (rcvid == 0) {
            switch (rx.pulse.code) {
            case PC_EMERGENCY: {
                motor_cmd_t c = {MOTOR_BRAKE, 0.0f};
                int rc = motor_apply(c);
                uint64_t t_applied = plat_now_ns();
                last = c;
                uint32_t seq = EMERG_SEQ(rx.pulse.value.sival_int);
                atomic_store(&g_emerg_seq_ack, seq);
                uint64_t t_det = atomic_load(&g_emerg_t_detect_ns);
                double lat_us = t_det ? (double)(t_applied - t_det) / 1000.0 : -1.0;
                aeb_state_t st = EMERG_STATE(rx.pulse.value.sival_int);
                log_event(lat_us > g_params.braking_deadline_us ? SEV_CRIT : SEV_WARN,
                          "EMERGENCY BRAKE applied (%s) seq=%u decision->outputs %.0f us "
                          "(deadline %d us)%s",
                          aeb_state_name(st), (unsigned)seq, lat_us, g_params.braking_deadline_us,
                          rc ? " GPIO ERROR" : "");
                if (lat_us > g_params.braking_deadline_us) fault_set(FAULT_DEADLINE_MISS);
                aeb_status_t s;
                shared_aeb_get(&s);
                float v_ref = s.threat.valid ? s.threat.v_close_mps : (s.ego_valid ? s.ego_speed_mps : 0.0f);
                if (brake_sup_on_state(&bs, st, t_applied, v_ref, s.imu_ok, &g_params))
                    verify_reported = false;
                cur_state = st;
                break;
            }
            case PC_FAULT_SAFE: {
                motor_cmd_t c = {MOTOR_BRAKE, 0.0f};
                motor_apply(c);
                last = c;
                break;
            }
            case PC_TIMER: {
                hb_beat(HB_BRAKE);
                imu_state_t im;
                shared_imu_get(&im);
                bool imu_ok = imu_is_fresh(&im, now, &g_params);
                int st = brake_sup_tick(&bs, now, im.a_long, imu_ok, &g_params);
                if (!verify_reported && st != VERIFY_RUNNING) {
                    verify_reported = true;
                    if (st == VERIFY_FAIL) {
                        fault_set(FAULT_BRAKE_NO_DECEL);
                        log_event(SEV_CRIT,
                                  "BRAKE VERIFICATION FAIL: %s level, peak decel %.2f m/s2 < %.2f "
                                  "within %d ms -> MOTOR/BRAKE FAULT",
                                  aeb_state_name(cur_state), bs.bv.peak_decel, bs.bv.required,
                                  g_params.verify_window_ms);
                    } else if (st == VERIFY_PASS) {
                        log_event(SEV_INFO, "brake verified: peak decel %.2f m/s2 (>= %.2f) after %.0f ms",
                                  bs.bv.peak_decel, bs.bv.required, (double)(now - bs.bv.t_cmd_ns) / 1e6);
                    } else if (st == VERIFY_SKIPPED) {
                        log_event(SEV_WARN, "brake verification SKIPPED (speed %.2f m/s or IMU unavailable)",
                                  bs.bv.v_ref_mps);
                    }
                }
                break;
            }
            case PC_SHUTDOWN: timer_delete(tid); return NULL;
            default: break;
            }
            continue;
        }
        if (rx.type == MSG_BRAKE_DECISION) {
            brake_reply_t rep;
            memset(&rep, 0, sizeof rep);
            rep.seq = rx.dec.seq;
            motor_cmd_t c = rx.dec.cmd;
            if (!same_cmd(c, last) || motor_safe_latched()) {
                rep.status = motor_apply(c) == 0 ? 0 : EIO;
                last = motor_get_applied();
            }
            rep.t_applied_ns = plat_now_ns();
            aeb_state_t st = (aeb_state_t)rx.dec.state;
            if (brake_sup_on_state(&bs, st, rep.t_applied_ns, rx.dec.v_ref_mps, rx.dec.imu_ok != 0,
                                   &g_params))
                verify_reported = false;
            cur_state = st;
            MsgReply(rcvid, EOK, &rep, sizeof rep);
        } else {
            MsgError(rcvid, ENOSYS);
        }
    }
}
