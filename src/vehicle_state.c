#include "vehicle_state.h"
#include "aeb_logic.h"
#include "aeb_cycle.h"
#include "camera_if.h"
#include "telemetry.h"
#include "motor_bts7960.h"
#include "shared_state.h"
#include "platform_qnx.h"
#include "logger.h"
#include "params.h"
#include "config.h"
#include <string.h>
_Atomic float g_sim_ego_speed;
const char *ego_src_name(ego_src_t s)
{
    switch (s) {
    case EGO_SRC_NONE: return "none";
    case EGO_SRC_RADAR_STATIC: return "radar_static";
    case EGO_SRC_IMU: return "imu";
    case EGO_SRC_ENCODER: return "encoder";
    case EGO_SRC_SIM: return "sim";
    default: return "?";
    }
}
int ego_src_parse(const char *s, ego_src_t *out)
{
    static const ego_src_t all[] = {EGO_SRC_NONE, EGO_SRC_RADAR_STATIC, EGO_SRC_IMU, EGO_SRC_ENCODER,
                                    EGO_SRC_SIM};
    for (unsigned i = 0; i < sizeof all / sizeof all[0]; i++)
        if (strcmp(s, ego_src_name(all[i])) == 0) {
            *out = all[i];
            return 0;
        }
    return -1;
}
static bool encoder_get_speed(float *mps)
{
    (void)mps;
    return false;
}
void *vehicle_thread(void *arg)
{
    (void)arg;
    vehicle_state_t vs;
    memset(&vs, 0, sizeof vs);
    vs.src = g_ego_src;
    stop_det_t sd;
    memset(&sd, 0, sizeof sd);
    float v_imu = 0.0f;
    bool imu_int_valid = false;
    uint64_t last = plat_now_ns(), next = last, last_zupt = last;
    int cam_link = -1;
    int cam_last_class = -1;
    uint64_t cam_last_class_log = 0;
    aeb_state_t prev_aeb = AEB_NORMAL;
    if (g_ego_src == EGO_SRC_RADAR_STATIC)
        log_event(SEV_WARN, "ego speed source = radar_static: ASSUMES the in-path obstacle is stationary");
    else if (g_ego_src == EGO_SRC_NONE)
        log_event(SEV_INFO, "ego speed source = none (no encoder): ego speed reported invalid");
    while (atomic_load(&g_running)) {
        hb_beat(HB_VEHICLE);
        uint64_t now = plat_now_ns();
        float dt = (float)(now - last) * 1e-9f;
        last = now;
        imu_state_t im;
        shared_imu_get(&im);
        aeb_status_t st;
        shared_aeb_get(&st);
        motor_cmd_t cmd = motor_get_applied();
        bool imu_ok = imu_is_fresh(&im, now, &g_params);
        stop_det_t sd2 = sd;
        bool quiet_rest = stop_detect_update(&sd2, now, false, 0.0f, imu_ok, im.a_long, cmd.type, &g_params);
        sd = sd2;
        vs.ego_valid = false;
        switch (g_ego_src) {
        case EGO_SRC_RADAR_STATIC:
            if (st.threat.valid) {
                vs.ego_speed_mps = st.threat.v_close_mps > 0 ? st.threat.v_close_mps : 0.0f;
                vs.ego_valid = true;
            }
            break;
        case EGO_SRC_IMU:
            if (imu_ok) {
                if (quiet_rest) {
                    v_imu = 0.0f;
                    imu_int_valid = true;
                    last_zupt = now;
                } else if (imu_int_valid) {
                    v_imu += im.a_long * dt;
                    if (v_imu < 0) v_imu = 0;
                }
                if (imu_int_valid && now - last_zupt < 5000000000ull) {
                    vs.ego_speed_mps = v_imu;
                    vs.ego_valid = true;
                }
            } else {
                imu_int_valid = false;
            }
            break;
        case EGO_SRC_ENCODER: {
            float v;
            if (encoder_get_speed(&v)) {
                vs.ego_speed_mps = v;
                vs.ego_valid = true;
            }
            break;
        }
        case EGO_SRC_SIM:
            vs.ego_speed_mps = atomic_load(&g_sim_ego_speed);
            vs.ego_valid = true;
            break;
        case EGO_SRC_NONE:
        default: break;
        }
        vs.stopped = vs.ego_valid ? (vs.ego_speed_mps < g_params.v_stopped_mps) : quiet_rest;
        vs.src = g_ego_src;
        vs.t_ns = now;
        shared_veh_put(&vs);
        camera_det_t cd;
        camera_get_latest_detection(&cd);
        int link = camera_connected() ? 1 : 0;
        telemetry_camera(&cd, link);
        if (link != cam_link) {
            if (link) {
                fault_clear(FAULT_CAMERA_LOST);
                log_event(
                    SEV_INFO,
                    "camera (aeb_vision) connected (inference %.0f ms) - complementary classification only",
                    (double)cd.infer_ms);
            } else {
                fault_set(FAULT_CAMERA_LOST);
                log_event(SEV_INFO, "camera (aeb_vision) not connected - radar-only AEB");
            }
            cam_link = link;
        }
        if (link) {
            int cls = cd.valid ? cd.class_id : -1;
            if (cls != cam_last_class && now - cam_last_class_log > 1000000000ull) {
                if (cd.valid) {
                    char up[16];
                    size_t i = 0;
                    for (; cd.label[i] && i < sizeof up - 1; i++)
                        up[i] = (cd.label[i] >= 'a' && cd.label[i] <= 'z') ? (char)(cd.label[i] - 32)
                                                                           : cd.label[i];
                    up[i] = '\0';
                    log_event(SEV_INFO, "OBJECT DETECTED: %s (camera, conf %.2f, bearing %+.0f deg, ahead)",
                              up, (double)cd.confidence, (double)cd.bearing_deg);
                } else {
                    log_event(SEV_INFO, "camera: no classified object ahead");
                }
                cam_last_class = cls;
                cam_last_class_log = now;
            }
        }
        {
            aeb_status_t st;
            shared_aeb_get(&st);
            bool braking = st.state == AEB_PARTIAL_BRAKE || st.state == AEB_STRONG_BRAKE ||
                           st.state == AEB_EMERGENCY_BRAKE;
            bool was_braking = prev_aeb == AEB_PARTIAL_BRAKE || prev_aeb == AEB_STRONG_BRAKE ||
                               prev_aeb == AEB_EMERGENCY_BRAKE;
            if (braking && !was_braking) {
                if (!link)
                    log_event(SEV_INFO, "%s: radar-only (camera not connected)", aeb_state_name(st.state));
                else if (cd.valid)
                    log_event(SEV_INFO,
                              "%s: camera CONFIRMS %s (conf %.2f, bearing %+.0f deg), radar range %.2f m",
                              aeb_state_name(st.state), cd.label, (double)cd.confidence,
                              (double)cd.bearing_deg, (double)st.threat.range_m);
                else
                    log_event(
                        SEV_INFO,
                        "%s: camera does not classify the radar target (radar range %.2f m) - braking anyway",
                        aeb_state_name(st.state), (double)st.threat.range_m);
            }
            prev_aeb = st.state;
        }
        next += (uint64_t)VEHICLE_PERIOD_MS * 1000000ull;
        if (next < now) next = now;
        plat_sleep_until_ns(next);
    }
    return NULL;
}
