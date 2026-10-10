#include "shared_state.h"
#include <pthread.h>
#include <string.h>
atomic_uint g_hb[HB_COUNT];
atomic_bool g_hb_critical[HB_COUNT];
atomic_bool g_hb_enabled[HB_COUNT];
atomic_bool g_running = true;
atomic_bool g_cmd_go;
atomic_bool g_cmd_stop;
atomic_uint g_wdog_tripped_mask;
run_mode_t g_mode = MODE_FULL;
ego_src_t g_ego_src = EGO_SRC_NONE;
int g_verbose;
atomic_uint g_emerg_seq_sent;
atomic_uint g_emerg_seq_ack;
_Atomic uint64_t g_emerg_t_detect_ns;
static atomic_uint g_faults;
static pthread_mutex_t m_radar, m_imu, m_veh, m_aeb;
static radar_state_t s_radar;
static imu_state_t s_imu;
static vehicle_state_t s_veh;
static aeb_status_t s_aeb;
const char *hb_name(hb_id_t id)
{
    static const char *const n[HB_COUNT] = {"radar",   "imu",    "prediction", "brake", "motor",
                                            "vehicle", "logger", "watchdog",   "sim"};
    return ((unsigned)id < HB_COUNT) ? n[id] : "?";
}
void hb_enable(hb_id_t id, bool critical)
{
    atomic_store(&g_hb_critical[id], critical);
    atomic_store(&g_hb_enabled[id], true);
}
void fault_set(uint32_t bits)
{
    atomic_fetch_or(&g_faults, bits);
}
uint32_t fault_get(void)
{
    return atomic_load(&g_faults);
}
void fault_clear(uint32_t bits)
{
    atomic_fetch_and(&g_faults, ~bits);
}
static void mtx_init(pthread_mutex_t *m)
{
    pthread_mutexattr_t a;
    pthread_mutexattr_init(&a);
    pthread_mutexattr_setprotocol(&a, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(m, &a);
    pthread_mutexattr_destroy(&a);
}
void shared_init(void)
{
    mtx_init(&m_radar);
    mtx_init(&m_imu);
    mtx_init(&m_veh);
    mtx_init(&m_aeb);
    memset(&s_radar, 0, sizeof s_radar);
    memset(&s_imu, 0, sizeof s_imu);
    memset(&s_veh, 0, sizeof s_veh);
    memset(&s_aeb, 0, sizeof s_aeb);
    s_aeb.state = AEB_STOPPED;
}
#define PUTGET(name, type, mtx, var)                                                                         \
    void shared_##name##_put(const type *x)                                                                  \
    {                                                                                                        \
        pthread_mutex_lock(&mtx);                                                                            \
        var = *x;                                                                                            \
        pthread_mutex_unlock(&mtx);                                                                          \
    }                                                                                                        \
    void shared_##name##_get(type *x)                                                                        \
    {                                                                                                        \
        pthread_mutex_lock(&mtx);                                                                            \
        *x = var;                                                                                            \
        pthread_mutex_unlock(&mtx);                                                                          \
    }
PUTGET(radar, radar_state_t, m_radar, s_radar)
PUTGET(imu, imu_state_t, m_imu, s_imu)
PUTGET(veh, vehicle_state_t, m_veh, s_veh)
PUTGET(aeb, aeb_status_t, m_aeb, s_aeb)
