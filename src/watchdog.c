#include "watchdog.h"
#include "ipc.h"
#include "brake.h"
#include "motor_bts7960.h"
#include "shared_state.h"
#include "platform_qnx.h"
#include "logger.h"
#include "params.h"
#include "config.h"
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
static int g_chid = -1, g_coid = -1;
static timer_t g_deadline_tid;
static int g_guard_coid = -1;
static atomic_uint g_deadline_seq;
bool watchdog_guard_connected(void)
{
    return g_guard_coid >= 0;
}
int watchdog_init(bool require_guard)
{
    if (plat_channel_create(&g_chid, &g_coid) != 0) return -1;
    if (plat_timer_oneshot(g_coid, PC_DEADLINE, PRIO_WATCHDOG, &g_deadline_tid) != 0) return -1;
    g_guard_coid = name_open(GUARD_NAME, 0);
    if (g_guard_coid == -1) {
        fault_set(FAULT_GUARD_MISSING);
        if (require_guard) return -2;
        return 0;
    }
    guard_msg_t m = {MSG_GUARD_REGISTER, 0, (int32_t)getpid()};
    if (MsgSend(g_guard_coid, &m, sizeof m, NULL, 0) == -1) {
        name_close(g_guard_coid);
        g_guard_coid = -1;
        fault_set(FAULT_GUARD_MISSING);
        if (require_guard) return -2;
    }
    return 0;
}
void watchdog_arm_deadline(uint32_t seq)
{
    atomic_store(&g_deadline_seq, seq);
    plat_timer_arm_us(g_deadline_tid, (uint32_t)g_params.braking_deadline_us);
}
void watchdog_stop(void)
{
    if (g_coid >= 0) MsgSendPulse(g_coid, PRIO_WATCHDOG, PC_SHUTDOWN, 0);
}
void watchdog_guard_clean_exit(void)
{
    if (g_guard_coid < 0) return;
    guard_msg_t m = {MSG_GUARD_CLEAN_EXIT, 0, (int32_t)getpid()};
    MsgSend(g_guard_coid, &m, sizeof m, NULL, 0);
    name_close(g_guard_coid);
    g_guard_coid = -1;
}
static void trip(const char *why, uint32_t fault)
{
    fault_set(fault);
    motor_force_safe(true, why);
    if (brake_coid() >= 0) MsgSendPulse(brake_coid(), PRIO_EMERGENCY_PULSE, PC_FAULT_SAFE, 0);
}
void *watchdog_thread(void *arg)
{
    (void)arg;
    timer_t tid;
    if (plat_timer_periodic(g_coid, PC_TIMER, PRIO_WATCHDOG, WATCHDOG_PERIOD_MS * 1000u, &tid) != 0) {
        log_event(SEV_CRIT, "watchdog timer failed (%s) -> safe state", strerror(errno));
        trip("watchdog timer failure", FAULT_WDOG_THREAD);
        return NULL;
    }
    uint32_t last_hb[HB_COUNT];
    uint64_t t_seen[HB_COUNT];
    bool reported[HB_COUNT];
    uint64_t now = plat_now_ns();
    for (int i = 0; i < HB_COUNT; i++) {
        last_hb[i] = atomic_load(&g_hb[i]);
        t_seen[i] = now;
        reported[i] = false;
    }
    uint64_t last_guard = 0;
    const uint64_t to_ns = (uint64_t)g_params.watchdog_timeout_ms * 1000000ull;
    const uint64_t stale_drive_ns =
        (uint64_t)(g_params.radar_timeout_ms + g_params.watchdog_timeout_ms) * 1000000ull;
    while (atomic_load(&g_running)) {
        struct _pulse p;
        if (MsgReceivePulse(g_chid, &p, sizeof p, NULL) == -1) continue;
        now = plat_now_ns();
        if (p.code == PC_SHUTDOWN) break;
        if (p.code == PC_DEADLINE) {
            uint32_t want = atomic_load(&g_deadline_seq);
            uint32_t ack = atomic_load(&g_emerg_seq_ack);
            if ((int32_t)(ack - want) < 0) {
                log_event(SEV_CRIT, "BRAKING DEADLINE MISSED: seq %u not applied within %d us",
                          (unsigned)want, g_params.braking_deadline_us);
                trip("braking deadline missed", FAULT_DEADLINE_MISS);
            }
            continue;
        }
        if (p.code != PC_TIMER) continue;
        hb_beat(HB_WATCHDOG);
        for (int i = 0; i < HB_COUNT; i++) {
            if (!atomic_load(&g_hb_enabled[i]) || i == HB_WATCHDOG) continue;
            uint32_t hb = atomic_load(&g_hb[i]);
            if (hb != last_hb[i]) {
                last_hb[i] = hb;
                t_seen[i] = now;
                if (reported[i]) {
                    log_event(SEV_WARN, "watchdog: %s thread alive again (fault stays latched)",
                              hb_name((hb_id_t)i));
                    reported[i] = false;
                }
                continue;
            }
            if (now - t_seen[i] > to_ns && !reported[i]) {
                reported[i] = true;
                if (atomic_load(&g_hb_critical[i])) {
                    atomic_fetch_or(&g_wdog_tripped_mask, 1u << i);
                    log_event(SEV_CRIT, "WATCHDOG: critical thread '%s' silent for %.0f ms -> SYSTEM FAULT",
                              hb_name((hb_id_t)i), (double)(now - t_seen[i]) / 1e6);
                    trip(hb_name((hb_id_t)i), FAULT_WDOG_THREAD);
                } else {
                    log_event(SEV_WARN, "watchdog: non-critical thread '%s' silent for %.0f ms",
                              hb_name((hb_id_t)i), (double)(now - t_seen[i]) / 1e6);
                }
            }
        }
        if (atomic_load(&g_hb_enabled[HB_PREDICT])) {
            motor_cmd_t c = motor_get_applied();
            radar_state_t rs;
            shared_radar_get(&rs);
            if (c.type == MOTOR_DRIVE && !motor_safe_latched() &&
                (rs.t_ns == 0 || age_ns(now, rs.t_ns) > stale_drive_ns)) {
                log_event(SEV_CRIT, "WATCHDOG: driving on stale radar data (%.0f ms) -> SYSTEM FAULT",
                          rs.t_ns ? (double)age_ns(now, rs.t_ns) / 1e6 : -1.0);
                trip("driving on stale radar", FAULT_WDOG_THREAD);
            }
        }
        if (g_guard_coid >= 0 && now - last_guard >= (uint64_t)GUARD_HEARTBEAT_MS * 1000000ull) {
            if (MsgSendPulse(g_guard_coid, PRIO_GUARD, PC_GUARD_HB, 0) == -1) {
                log_event(SEV_ERROR, "guard heartbeat failed (%s) - guard lost", strerror(errno));
                name_close(g_guard_coid);
                g_guard_coid = -1;
                fault_set(FAULT_GUARD_MISSING);
            }
            last_guard = now;
        }
    }
    timer_delete(tid);
    return NULL;
}
