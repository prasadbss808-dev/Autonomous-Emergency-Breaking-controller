#include "imu_proc.h"
#include "aeb_logic.h"
#include <string.h>
void imu_proc_init(imu_proc_t *ip)
{
    memset(ip, 0, sizeof(*ip));
    ip->g_axis = 2;
}
void imu_proc_cal_add(imu_proc_t *ip, const float v[6], float max_gyro_dps)
{
    for (int i = 0; i < 6; i++) {
        ip->sum[i] += v[i];
        ip->sumsq[i] += (double)v[i] * v[i];
    }
    ip->cal_max_gyro_dps = max_gyro_dps;
    ip->n_cal++;
}
bool imu_proc_cal_finish(imu_proc_t *ip, int min_samples)
{
    if (ip->n_cal < min_samples) return false;
    float mean[6];
    for (int i = 0; i < 6; i++) mean[i] = (float)(ip->sum[i] / ip->n_cal);
    for (int i = 3; i < 6; i++) {
        double var = ip->sumsq[i] / ip->n_cal - (double)mean[i] * mean[i];
        if (var > (double)ip->cal_max_gyro_dps * ip->cal_max_gyro_dps) return false;
    }
    int ga = 0;
    float best = 0;
    for (int i = 0; i < 3; i++) {
        float a = mean[i] < 0 ? -mean[i] : mean[i];
        if (a > best) {
            best = a;
            ga = i;
        }
    }
    if (best < 0.7f * G_MPS2 || best > 1.3f * G_MPS2) return false;
    for (int i = 3; i < 6; i++) {
        float g = mean[i] < 0 ? -mean[i] : mean[i];
        if (g > 25.0f) return false;
    }
    ip->g_axis = ga;
    for (int i = 0; i < 6; i++) ip->bias[i] = mean[i];
    ip->calibrated = true;
    ip->lpf_init = false;
    return true;
}
void imu_proc_cal_set_zero(imu_proc_t *ip)
{
    memset(ip->bias, 0, sizeof ip->bias);
    ip->calibrated = true;
    ip->lpf_init = false;
}
void imu_proc_clear_impact(imu_proc_t *ip)
{
    ip->impact_latched = false;
    ip->impact_cnt = 0;
}
void imu_proc_sample(imu_proc_t *ip, const float v[6], uint64_t t_ns, const aeb_params_t *p, imu_state_t *out)
{
    float c[6];
    for (int i = 0; i < 6; i++) c[i] = v[i] - ip->bias[i];
    if (!ip->lpf_init) {
        memcpy(ip->lpf, c, sizeof c);
        ip->lpf_init = true;
    } else
        for (int i = 0; i < 6; i++) ip->lpf[i] += p->imu_lpf_alpha * (c[i] - ip->lpf[i]);
    const float thr = p->impact_threshold_g * G_MPS2;
    const float m2 = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
    if (m2 >= thr * thr) {
        if (++ip->impact_cnt >= p->impact_debounce_n) ip->impact_latched = true;
    } else {
        ip->impact_cnt = 0;
    }
    const int fa = (p->imu_forward_axis >= 0 && p->imu_forward_axis <= 2) ? p->imu_forward_axis : 0;
    const float sgn = (p->imu_forward_sign < 0) ? -1.0f : 1.0f;
    out->t_ns = t_ns;
    out->ax = ip->lpf[0];
    out->ay = ip->lpf[1];
    out->az = ip->lpf[2];
    out->gx = ip->lpf[3];
    out->gy = ip->lpf[4];
    out->gz = ip->lpf[5];
    out->a_long = sgn * ip->lpf[fa];
    out->a_long_raw = sgn * c[fa];
    out->a_dyn_g = aeb_sqrtf(m2) / G_MPS2;
    out->impact = ip->impact_latched;
    out->calibrated = ip->calibrated;
    out->samples++;
    out->valid = ip->calibrated;
}
