#ifndef SHARED_STATE_H
#define SHARED_STATE_H
#include <stdatomic.h>
#include "aeb_types.h"
typedef enum {
    HB_RADAR = 0,
    HB_IMU,
    HB_PREDICT,
    HB_BRAKE,
    HB_MOTOR,
    HB_VEHICLE,
    HB_LOGGER,
    HB_WATCHDOG,
    HB_SIM,
    HB_COUNT
} hb_id_t;
typedef enum {
    MODE_FULL = 0,
    MODE_SENSOR,
    MODE_RADAR,
    MODE_IMU,
    MODE_MOTOR,
    MODE_CALIB,
    MODE_SIM,
    MODE_SELFTEST,
    MODE_I2CSCAN
} run_mode_t;
extern atomic_uint g_hb[HB_COUNT];
extern atomic_bool g_hb_critical[HB_COUNT];
extern atomic_bool g_hb_enabled[HB_COUNT];
extern atomic_bool g_running;
extern atomic_bool g_cmd_go;
extern atomic_bool g_cmd_stop;
extern atomic_uint g_wdog_tripped_mask;
extern run_mode_t g_mode;
extern ego_src_t g_ego_src;
extern int g_verbose;
extern atomic_uint g_emerg_seq_sent;
extern atomic_uint g_emerg_seq_ack;
extern _Atomic uint64_t g_emerg_t_detect_ns;
const char *hb_name(hb_id_t id);
static inline void hb_beat(hb_id_t id)
{
    atomic_fetch_add_explicit(&g_hb[id], 1u, memory_order_relaxed);
}
void hb_enable(hb_id_t id, bool critical);
void fault_set(uint32_t bits);
uint32_t fault_get(void);
void fault_clear(uint32_t bits);
void shared_init(void);
void shared_radar_put(const radar_state_t *r);
void shared_radar_get(radar_state_t *r);
void shared_imu_put(const imu_state_t *m);
void shared_imu_get(imu_state_t *m);
void shared_veh_put(const vehicle_state_t *v);
void shared_veh_get(vehicle_state_t *v);
void shared_aeb_put(const aeb_status_t *s);
void shared_aeb_get(aeb_status_t *s);
#endif
