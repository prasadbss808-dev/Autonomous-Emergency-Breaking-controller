#include "sim_model.h"
#include "aeb_logic.h"
#include <string.h>
#define PATH_HALF_WIDTH_M 0.15f
static const sim_scenario_t k_sc[] = {
    {1,
     "No obstacle",
     "NORMAL driving throughout",
     1.2f,
     5.0f,
     0,
     {{0.0f, 0.0f, 0.0f, 0.0f}},
     SIMF_NONE,
     0,
     AEB_NORMAL,
     AEB_NORMAL,
     AEB_NORMAL,
     true,
     0,
     false},
    {2,
     "Obstacle far away (20 m)",
     "stays NORMAL",
     1.2f,
     5.0f,
     1,
     {{0.0f, 20.0f, 0.0f, 0.0f}},
     SIMF_NONE,
     0,
     AEB_NORMAL,
     AEB_NORMAL,
     AEB_NORMAL,
     true,
     0,
     false},
    {3,
     "Obstacle approaching slowly (lead car 0.8 m/s)",
     "WARNING / PARTIAL, no collision",
     1.2f,
     10.0f,
     1,
     {{0.0f, 2.5f, 0.8f, 0.0f}},
     SIMF_NONE,
     0,
     AEB_WARNING,
     AEB_PARTIAL_BRAKE,
     -1,
     true,
     0,
     false},
    {4,
     "Obstacle approaching rapidly (lead car 0.1 m/s)",
     "STRONG braking, no collision",
     1.2f,
     6.0f,
     1,
     {{0.0f, 3.0f, 0.1f, 0.0f}},
     SIMF_NONE,
     0,
     AEB_STRONG_BRAKE,
     AEB_EMERGENCY_BRAKE,
     -1,
     true,
     0,
     false},
    {5,
     "Very low TTC (obstacle appears 0.55 m ahead)",
     "EMERGENCY brake -> STOPPED",
     1.2f,
     4.0f,
     1,
     {{0.0f, 1.75f, 0.0f, 1.0f}},
     SIMF_NONE,
     0,
     AEB_EMERGENCY_BRAKE,
     AEB_EMERGENCY_BRAKE,
     AEB_STOPPED,
     true,
     0,
     false},
    {6,
     "Radar timeout",
     "RADAR_FAULT -> safe brake",
     1.2f,
     4.0f,
     0,
     {{0.0f, 0.0f, 0.0f, 0.0f}},
     SIMF_RADAR_TIMEOUT,
     2.0f,
     AEB_NORMAL,
     AEB_NORMAL,
     AEB_RADAR_FAULT,
     true,
     FAULT_RADAR_TIMEOUT,
     false},
    {7,
     "IMU failure",
     "IMU_FAULT -> safe brake (policy 0)",
     1.2f,
     4.0f,
     0,
     {{0.0f, 0.0f, 0.0f, 0.0f}},
     SIMF_IMU_FAIL,
     2.0f,
     AEB_NORMAL,
     AEB_NORMAL,
     AEB_IMU_FAULT,
     true,
     FAULT_IMU_TIMEOUT,
     false},
    {8,
     "Brake commanded but no deceleration",
     "MOTOR_FAULT (brake verification FAIL)",
     1.2f,
     5.0f,
     1,
     {{0.0f, 4.0f, 0.0f, 0.0f}},
     SIMF_MOTOR_NO_DECEL,
     0.0f,
     AEB_WARNING,
     AEB_EMERGENCY_BRAKE,
     AEB_MOTOR_FAULT,
     false,
     FAULT_BRAKE_NO_DECEL,
     false},
    {9,
     "Invalid radar range / malformed frames",
     "measurements rejected, stays NORMAL",
     1.2f,
     4.0f,
     0,
     {{0.0f, 0.0f, 0.0f, 0.0f}},
     SIMF_INVALID_RANGE,
     0.5f,
     AEB_NORMAL,
     AEB_NORMAL,
     AEB_NORMAL,
     true,
     0,
     true},
    {10,
     "Multiple targets (2 beside path, 1 in path)",
     "brakes for the IN-PATH target, no collision",
     1.2f,
     6.0f,
     3,
     {{0.7f, 1.5f, 0.0f, 0.0f}, {0.0f, 3.0f, 0.0f, 0.0f}, {-1.0f, 2.0f, 0.0f, 0.0f}},
     SIMF_NONE,
     0,
     AEB_STRONG_BRAKE,
     AEB_EMERGENCY_BRAKE,
     -1,
     true,
     0,
     false},
    {11,
     "Targets outside collision path only",
     "no braking (stays NORMAL)",
     1.2f,
     5.0f,
     2,
     {{0.6f, 2.0f, 0.0f, 0.0f}, {-0.5f, 1.2f, 0.0f, 0.0f}},
     SIMF_NONE,
     0,
     AEB_NORMAL,
     AEB_NORMAL,
     AEB_NORMAL,
     true,
     0,
     false},
    {12,
     "Watchdog timeout (critical thread hangs)",
     "SYSTEM_FAULT -> safe state",
     1.2f,
     4.0f,
     0,
     {{0.0f, 0.0f, 0.0f, 0.0f}},
     SIMF_WATCHDOG_HANG,
     2.0f,
     AEB_NORMAL,
     AEB_NORMAL,
     AEB_SYSTEM_FAULT,
     true,
     FAULT_WDOG_THREAD,
     false},
};
int sim_num_scenarios(void)
{
    return (int)(sizeof k_sc / sizeof k_sc[0]);
}
const sim_scenario_t *sim_get_scenario(int id)
{
    for (int i = 0; i < sim_num_scenarios(); i++)
        if (k_sc[i].id == id) return &k_sc[i];
    return NULL;
}
void sim_world_init(sim_world_t *w, const sim_scenario_t *sc)
{
    memset(w, 0, sizeof(*w));
    w->v_ego_mps = sc->v0_mps;
    w->min_gap_m = 1e9f;
}
static float obj_y(const sim_obj_t *o, float t)
{
    return o->y0_m + o->v_mps * t;
}
static bool obj_exists(const sim_obj_t *o, float t)
{
    return t >= o->appear_t_s;
}
static float fabs_(float v)
{
    return v < 0 ? -v : v;
}
void sim_world_step(sim_world_t *w, const sim_scenario_t *sc, motor_cmd_t cmd, float dt)
{
    float a;
    bool no_effect = (sc->fault == SIMF_MOTOR_NO_DECEL && w->t_s >= sc->t_fault_s);
    if (no_effect) {
        a = 0.0f;
    } else if (cmd.type == MOTOR_DRIVE) {
        a = ((cmd.duty_pct / 100.0f) * SIM_V_MAX_MPS - w->v_ego_mps) / SIM_TAU_S;
        if (a < -SIM_DRIVE_DECEL_MAX) a = -SIM_DRIVE_DECEL_MAX;
    } else if (cmd.type == MOTOR_BRAKE) {
        a = -SIM_BRAKE_DECEL;
    } else {
        a = -SIM_COAST_DECEL;
    }
    if (w->v_ego_mps <= 0.0f && a < 0.0f) a = 0.0f;
    w->v_ego_mps += a * dt;
    if (w->v_ego_mps < 0.0f) {
        w->v_ego_mps = 0.0f;
    }
    w->a_ego_mps2 = a;
    w->x_ego_m += w->v_ego_mps * dt;
    w->t_s += dt;
    for (int i = 0; i < sc->n_obj; i++) {
        const sim_obj_t *o = &sc->obj[i];
        if (!obj_exists(o, w->t_s) || fabs_(o->x_lat_m) > PATH_HALF_WIDTH_M) continue;
        float gap = obj_y(o, w->t_s) - w->x_ego_m;
        if (gap < w->min_gap_m) w->min_gap_m = gap;
        if (gap <= 0.0f) w->collided = true;
    }
}
static int16_t q16(float v)
{
    float r = v >= 0 ? v + 0.5f : v - 0.5f;
    if (r > 32767.0f) r = 32767.0f;
    if (r < -32767.0f) r = -32767.0f;
    return (int16_t)r;
}
void sim_make_radar_frame(const sim_world_t *w, const sim_scenario_t *sc, rd03d_frame_t *f, unsigned frame_no)
{
    (void)frame_no;
    memset(f, 0, sizeof(*f));
    int slot = 0;
    for (int i = 0; i < sc->n_obj && slot < RD03D_NUM_SLOTS; i++) {
        const sim_obj_t *o = &sc->obj[i];
        if (!obj_exists(o, w->t_s)) continue;
        float y = obj_y(o, w->t_s) - w->x_ego_m;
        if (y <= 0.05f) continue;
        float x = o->x_lat_m;
        float r = aeb_sqrtf(x * x + y * y);
        float v_long = o->v_mps - w->v_ego_mps;
        float v_rad = v_long * (y / r);
        if (fabs_(v_rad) < SIM_DOPPLER_MIN) continue;
        rd03d_slot_t *s = &f->slot[slot++];
        s->present = true;
        s->x_mm = q16(x * 1000.0f);
        s->y_mm = q16(y * 1000.0f);
        s->speed_cms = q16(v_rad * 100.0f);
        if (s->speed_cms == 0) s->speed_cms = v_rad < 0 ? -1 : 1;
        s->res_mm = 360;
    }
    if (sc->fault == SIMF_INVALID_RANGE && w->t_s >= sc->t_fault_s) {
        if (slot < RD03D_NUM_SLOTS) {
            rd03d_slot_t *s = &f->slot[slot++];
            s->present = true;
            s->x_mm = 0;
            s->y_mm = 30000;
            s->speed_cms = -100;
            s->res_mm = 360;
        }
        if (slot < RD03D_NUM_SLOTS) {
            rd03d_slot_t *s = &f->slot[slot++];
            s->present = true;
            s->x_mm = 10;
            s->y_mm = 50;
            s->speed_cms = -150;
            s->res_mm = 360;
        }
    }
}
bool sim_radar_silent(const sim_world_t *w, const sim_scenario_t *sc)
{
    return sc->fault == SIMF_RADAR_TIMEOUT && w->t_s >= sc->t_fault_s;
}
bool sim_imu_silent(const sim_world_t *w, const sim_scenario_t *sc)
{
    return sc->fault == SIMF_IMU_FAIL && w->t_s >= sc->t_fault_s;
}
bool sim_inject_corrupt_frame(const sim_world_t *w, const sim_scenario_t *sc, unsigned frame_no)
{
    return sc->fault == SIMF_INVALID_RANGE && w->t_s >= sc->t_fault_s && (frame_no % 4u) == 0u;
}
bool sim_hang_now(const sim_world_t *w, const sim_scenario_t *sc)
{
    return sc->fault == SIMF_WATCHDOG_HANG && w->t_s >= sc->t_fault_s;
}
