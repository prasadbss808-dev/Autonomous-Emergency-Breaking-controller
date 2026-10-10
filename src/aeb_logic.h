#ifndef AEB_LOGIC_H
#define AEB_LOGIC_H
#include "aeb_types.h"
#include "params.h"
float aeb_sqrtf(float x);
bool aeb_ttc(float range_m, float v_close_mps, float v_min_mps, float *ttc_s);
float aeb_stopping_distance(float v_mps, float t_delay_s, float decel_mps2);
float aeb_safe_distance(float v_mps, float t_delay_s, float decel_mps2, float sf);
typedef struct {
    bool ttc_valid;
    float ttc_s;
    float d_safe_m[3];
    bool unavoidable;
    risk_level_t level_ttc;
    risk_level_t level_dist;
    risk_level_t level;
} risk_eval_t;
risk_level_t aeb_classify(const threat_t *th, const aeb_params_t *p, float scale, risk_eval_t *ev);
typedef struct {
    aeb_state_t state;
    uint64_t t_enter_ns;
    bool downgrade_pending;
    uint64_t t_downgrade_ns;
    bool has_driven;
    uint32_t transitions;
} aeb_sm_t;
typedef struct {
    uint64_t now_ns;
    bool radar_ok;
    bool imu_ok;
    bool motor_ok;
    bool system_ok;
    bool impact;
    risk_level_t level_enter;
    risk_level_t level_exit;
    bool vehicle_stopped;
    bool cmd_go;
    bool cmd_stop;
} aeb_sm_input_t;
void aeb_sm_init(aeb_sm_t *sm, uint64_t now_ns);
aeb_state_t aeb_sm_step(aeb_sm_t *sm, const aeb_sm_input_t *in, const aeb_params_t *p);
motor_cmd_t aeb_motor_cmd_for_state(aeb_state_t s, bool has_driven, bool degraded, const aeb_params_t *p);
int aeb_state_brake_idx(aeb_state_t s);
enum { VERIFY_IDLE = 0, VERIFY_RUNNING, VERIFY_PASS, VERIFY_FAIL, VERIFY_SKIPPED };
typedef struct {
    int status;
    int lvl_idx;
    uint64_t t_cmd_ns;
    float v_ref_mps;
    float peak_decel;
    float required;
} brake_verify_t;
void brake_verify_start(brake_verify_t *bv, int lvl_idx, uint64_t now_ns, float v_ref_mps, bool imu_ok,
                        const aeb_params_t *p);
int brake_verify_update(brake_verify_t *bv, uint64_t now_ns, float a_long, bool imu_ok,
                        const aeb_params_t *p);
const char *verify_name(int st);
typedef struct {
    bool quiet;
    uint64_t t_quiet_ns;
} stop_det_t;
bool stop_detect_update(stop_det_t *sd, uint64_t now_ns, bool ego_valid, float ego_mps, bool imu_ok,
                        float a_long, motor_cmd_type_t cmd, const aeb_params_t *p);
typedef struct {
    aeb_state_t last_state;
    brake_verify_t bv;
} brake_sup_t;
void brake_sup_init(brake_sup_t *bs);
bool brake_sup_on_state(brake_sup_t *bs, aeb_state_t s, uint64_t now_ns, float v_ref_mps, bool imu_ok,
                        const aeb_params_t *p);
int brake_sup_tick(brake_sup_t *bs, uint64_t now_ns, float a_long, bool imu_ok, const aeb_params_t *p);
#endif
