#include "sim_rt.h"
#include "sim_model.h"
#include "radar_rd03d.h"
#include "imu_mpu6050.h"
#include "aeb_controller.h"
#include "vehicle_state.h"
#include "motor_bts7960.h"
#include "shared_state.h"
#include "platform_qnx.h"
#include "logger.h"
#include "params.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
atomic_int g_sim_result = -1;
static const sim_scenario_t *g_sc;
int sim_rt_init(int id)
{
    g_sc = sim_get_scenario(id);
    return g_sc ? 0 : -1;
}
static float fabs_(float v)
{
    return v < 0 ? -v : v;
}
void *sim_thread(void *arg)
{
    (void)arg;
    const sim_scenario_t *sc = g_sc;
    sim_world_t w;
    sim_world_init(&w, sc);
    log_event(SEV_INFO, "SIM TEST %d: %s  (expected: %s)", sc->id, sc->name, sc->expect_text);
    const float dt = 0.002f;
    uint64_t next = plat_now_ns();
    unsigned frame_no = 0, k = 0;
    bool hang_done = false;
    aeb_state_t max_active = AEB_NORMAL, final_state = AEB_STOPPED;
    uint32_t faults = 0;
    float max_abs_x = 0.0f;
    bool driven = false;
    radar_state_t rs;
    uint64_t max_late_ns = 0;
    while (atomic_load(&g_running)) {
        hb_beat(HB_SIM);
        k++;
        uint64_t now = plat_now_ns();
        if (now > next && now - next > max_late_ns) max_late_ns = now - next;
        motor_cmd_t cmd = motor_get_applied();
        sim_world_step(&w, sc, cmd, dt);
        atomic_store(&g_sim_ego_speed, w.v_ego_mps);
        aeb_status_t st;
        shared_aeb_get(&st);
        if (!driven && st.state == AEB_STOPPED && w.t_s < 1.0f) atomic_store(&g_cmd_go, true);
        if (st.state != AEB_STOPPED) driven = true;
        if (driven) {
            if (st.state <= AEB_EMERGENCY_BRAKE && st.state > max_active) max_active = st.state;
            faults |= st.faults | fault_get();
            if (st.threat.valid && fabs_(st.threat.x_m) > max_abs_x) max_abs_x = fabs_(st.threat.x_m);
            final_state = st.state;
        }
        if ((k * 2u) % SIM_RADAR_PERIOD_MS == 0 && !sim_radar_silent(&w, sc)) {
            rd03d_frame_t f;
            uint8_t b[RD03D_FRAME_LEN];
            sim_make_radar_frame(&w, sc, &f, frame_no);
            rd03d_encode_frame(&f, b);
            if (sim_inject_corrupt_frame(&w, sc, frame_no)) b[29] ^= 0xFF;
            frame_no++;
            radar_ingest(b, RD03D_FRAME_LEN, now);
        }
        if ((k * 2u) % IMU_PERIOD_MS == 0 && !sim_imu_silent(&w, sc)) {
            float v6[6] = {0, 0, 0, 0, 0, 0};
            int ax = g_params.imu_forward_axis;
            v6[ax] = (g_params.imu_forward_sign < 0 ? -1.0f : 1.0f) * w.a_ego_mps2;
            imu_ingest_phys(v6, now);
        }
        if (!hang_done && sim_hang_now(&w, sc)) {
            atomic_store(&g_inject_hang_ms, g_params.watchdog_timeout_ms * 3);
            hang_done = true;
        }
        if (w.t_s >= sc->duration_s) {
            shared_radar_get(&rs);
            bool ok = true;
            if (max_active < sc->max_active_min || max_active > sc->max_active_max) ok = false;
            if (sc->final_state >= 0 && final_state != (aeb_state_t)sc->final_state) ok = false;
            if (sc->require_no_collision && w.collided) ok = false;
            if ((faults & sc->require_faults) != sc->require_faults) ok = false;
            if (sc->require_rejects && rs.targets_rejected + rs.frames_bad == 0) ok = false;
            if (max_abs_x > g_params.corridor_half_width_m + 1e-3f) ok = false;
            log_event(ok ? SEV_INFO : SEV_ERROR,
                      "SIM TEST %d %s: max=%s final=%s %s min_gap=%.2f m faults=0x%04x rejected=%u", sc->id,
                      ok ? "PASS" : "FAIL", aeb_state_name(max_active), aeb_state_name(final_state),
                      w.collided ? "COLLISION" : "no-collision", w.min_gap_m > 1e8f ? -1.0f : w.min_gap_m,
                      (unsigned)faults, (unsigned)(rs.targets_rejected + rs.frames_bad));
            log_event(SEV_INFO, "sim thread worst wake-up latency %.2f ms", (double)max_late_ns / 1e6);
            atomic_store(&g_hb_enabled[HB_SIM], false);
            atomic_store(&g_sim_result, ok ? 0 : 1);
            break;
        }
        next += 2000000ull;
        if (next < now) next = now;
        plat_sleep_until_ns(next);
    }
    return NULL;
}
