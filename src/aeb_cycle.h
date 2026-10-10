#ifndef AEB_CYCLE_H
#define AEB_CYCLE_H
#include "aeb_logic.h"
#include "radar_track.h"
typedef struct {
    uint64_t now_ns;
    const radar_state_t *radar;
    const imu_state_t *imu;
    const vehicle_state_t *veh;
    uint32_t latched_faults;
    bool cmd_go;
    bool cmd_stop;
} aeb_cycle_in_t;
typedef struct {
    aeb_status_t status;
    motor_cmd_t cmd;
    uint32_t live_faults;
    bool state_changed;
    aeb_state_t prev_state;
    risk_eval_t eval;
} aeb_cycle_out_t;
bool radar_is_fresh(const radar_state_t *r, uint64_t now_ns, const aeb_params_t *p);
bool imu_is_fresh(const imu_state_t *m, uint64_t now_ns, const aeb_params_t *p);
void aeb_cycle(aeb_sm_t *sm, const aeb_params_t *p, const aeb_cycle_in_t *in, aeb_cycle_out_t *out);
#endif
