#ifndef SIM_MODEL_H
#define SIM_MODEL_H
#include "aeb_types.h"
#include "rd03d_proto.h"
#define SIM_V_MAX_MPS 2.0f
#define SIM_TAU_S 0.30f
#define SIM_BRAKE_DECEL 2.5f
#define SIM_DRIVE_DECEL_MAX 2.0f
#define SIM_COAST_DECEL 0.4f
#define SIM_DOPPLER_MIN 0.05f
#define SIM_RADAR_PERIOD_MS 50
#define SIM_MAX_OBJ 3
typedef enum {
    SIMF_NONE = 0,
    SIMF_RADAR_TIMEOUT,
    SIMF_IMU_FAIL,
    SIMF_MOTOR_NO_DECEL,
    SIMF_INVALID_RANGE,
    SIMF_WATCHDOG_HANG
} sim_fault_t;
typedef struct {
    float x_lat_m;
    float y0_m;
    float v_mps;
    float appear_t_s;
} sim_obj_t;
typedef struct {
    int id;
    const char *name;
    const char *expect_text;
    float v0_mps;
    float duration_s;
    int n_obj;
    sim_obj_t obj[SIM_MAX_OBJ];
    sim_fault_t fault;
    float t_fault_s;
    aeb_state_t max_active_min;
    aeb_state_t max_active_max;
    int final_state;
    bool require_no_collision;
    uint32_t require_faults;
    bool require_rejects;
} sim_scenario_t;
typedef struct {
    float t_s;
    float x_ego_m, v_ego_mps, a_ego_mps2;
    bool collided;
    float min_gap_m;
} sim_world_t;
int sim_num_scenarios(void);
const sim_scenario_t *sim_get_scenario(int id);
void sim_world_init(sim_world_t *w, const sim_scenario_t *sc);
void sim_world_step(sim_world_t *w, const sim_scenario_t *sc, motor_cmd_t cmd, float dt);
void sim_make_radar_frame(const sim_world_t *w, const sim_scenario_t *sc, rd03d_frame_t *f,
                          unsigned frame_no);
bool sim_radar_silent(const sim_world_t *w, const sim_scenario_t *sc);
bool sim_imu_silent(const sim_world_t *w, const sim_scenario_t *sc);
bool sim_inject_corrupt_frame(const sim_world_t *w, const sim_scenario_t *sc, unsigned frame_no);
bool sim_hang_now(const sim_world_t *w, const sim_scenario_t *sc);
#endif
