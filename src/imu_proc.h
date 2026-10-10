#ifndef IMU_PROC_H
#define IMU_PROC_H
#include "aeb_types.h"
#include "params.h"
#define G_MPS2 9.80665f
typedef struct {
    double sum[6];
    double sumsq[6];
    float cal_max_gyro_dps;
    int n_cal;
    float bias[6];
    int g_axis;
    bool calibrated;
    float lpf[6];
    bool lpf_init;
    int impact_cnt;
    bool impact_latched;
} imu_proc_t;
void imu_proc_init(imu_proc_t *ip);
void imu_proc_cal_add(imu_proc_t *ip, const float phys[6], float max_gyro_dps);
bool imu_proc_cal_finish(imu_proc_t *ip, int min_samples);
void imu_proc_cal_set_zero(imu_proc_t *ip);
void imu_proc_sample(imu_proc_t *ip, const float phys[6], uint64_t t_ns, const aeb_params_t *p,
                     imu_state_t *out);
void imu_proc_clear_impact(imu_proc_t *ip);
#endif
