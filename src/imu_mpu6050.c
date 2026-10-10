#include "imu_mpu6050.h"
#include "imu_proc.h"
#include "params.h"
#include "config.h"
#include "shared_state.h"
#include "platform_qnx.h"
#include "logger.h"
#include "ipc.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <devctl.h>
#include <hw/i2c.h>
#include <sys/neutrino.h>
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1 0x6B
#define REG_WHO_AM_I 0x75
#define ACCEL_LSB_PER_G 4096.0f
#define GYRO_LSB_PER_DPS 65.5f
static int g_fd = -1;
static int g_addr = IMU_ADDR;
static imu_proc_t g_ip;
static imu_state_t g_st;
static bool g_verbose_imu;
static atomic_bool g_clear_impact_req;
void imu_set_verbose(bool v)
{
    g_verbose_imu = v;
}
void imu_clear_impact(void)
{
    atomic_store(&g_clear_impact_req, true);
}
static int i2c_write_reg(uint8_t reg, uint8_t val)
{
    struct {
        i2c_send_t hdr;
        uint8_t data[2];
    } m;
    memset(&m, 0, sizeof m);
    m.hdr.slave.addr = (uint32_t)g_addr;
    m.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    m.hdr.len = 2;
    m.hdr.stop = 1;
    m.data[0] = reg;
    m.data[1] = val;
    int err = devctl(g_fd, DCMD_I2C_SEND, &m, sizeof(m.hdr) + 2, NULL);
    if (err != EOK) {
        errno = err;
        return -1;
    }
    return 0;
}
static int i2c_read_regs(uint8_t reg, uint8_t *out, int n)
{
    struct {
        i2c_sendrecv_t hdr;
        uint8_t data[16];
    } m;
    if (n < 1 || n > 16) {
        errno = EINVAL;
        return -1;
    }
    memset(&m, 0, sizeof m);
    m.hdr.slave.addr = (uint32_t)g_addr;
    m.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    m.hdr.send_len = 1;
    m.hdr.recv_len = (uint32_t)n;
    m.hdr.stop = 1;
    m.data[0] = reg;
    int err = devctl(g_fd, DCMD_I2C_SENDRECV, &m, sizeof(m.hdr) + (size_t)n, NULL);
    if (err != EOK) {
        errno = err;
        return -1;
    }
    memcpy(out, m.data, (size_t)n);
    return 0;
}
static int write_verify(uint8_t reg, uint8_t val)
{
    uint8_t rb = 0;
    if (i2c_write_reg(reg, val) != 0) return -1;
    if (i2c_read_regs(reg, &rb, 1) != 0) return -1;
    if (rb != val) {
        errno = EIO;
        return -1;
    }
    return 0;
}
int imu_init(const char *dev, int addr)
{
    imu_proc_init(&g_ip);
    memset(&g_st, 0, sizeof g_st);
    if (dev == NULL) {
        imu_proc_cal_set_zero(&g_ip);
        return 0;
    }
    g_addr = addr;
    g_fd = open(dev, O_RDWR);
    if (g_fd < 0) {
        fprintf(stderr, "[imu] open %s failed: %s\n", dev, strerror(errno));
        fault_set(FAULT_IMU_I2C);
        return -1;
    }
    uint8_t who = 0;
    if (i2c_read_regs(REG_WHO_AM_I, &who, 1) != 0) {
        int alt = (addr == 0x68) ? 0x69 : 0x68;
        fprintf(stderr, "[imu] no answer at 0x%02X on %s (%s) - trying 0x%02X\n", addr, dev, strerror(errno),
                alt);
        g_addr = alt;
        if (i2c_read_regs(REG_WHO_AM_I, &who, 1) != 0) {
            fprintf(stderr,
                    "[imu] no answer at 0x%02X either. Run: aeb -m i2cscan -i %s\n"
                    "      check: SDA->pin 3, SCL->pin 5, VCC->pin 1 (3.3V), GND->pin 9\n",
                    alt, dev);
            fault_set(FAULT_IMU_I2C);
            return -1;
        }
        fprintf(stderr, "[imu] MPU found at 0x%02X (AD0 is HIGH/floating - tie AD0 to GND for 0x68)\n", alt);
    }
    if (who != 0x68) {
        if (who == 0x70 || who == 0x71 || who == 0x72 || who == 0x73)
            fprintf(stderr,
                    "[imu] WHO_AM_I=0x%02X (MPU6500/9250-family clone) - register-compatible, continuing\n",
                    who);
        else {
            fprintf(stderr, "[imu] unexpected WHO_AM_I=0x%02X (expected 0x68)\n", who);
            fault_set(FAULT_IMU_IMPLAUSIBLE);
            return -1;
        }
    }
    if (i2c_write_reg(REG_PWR_MGMT_1, 0x80) != 0) goto fail;
    plat_sleep_ms(100);
    if (write_verify(REG_PWR_MGMT_1, 0x01) != 0) goto fail;
    plat_sleep_ms(10);
    if (write_verify(REG_SMPLRT_DIV, 0x04) != 0) goto fail;
    if (write_verify(REG_CONFIG, 0x03) != 0) goto fail;
    if (write_verify(REG_GYRO_CONFIG, 0x08) != 0) goto fail;
    if (write_verify(REG_ACCEL_CONFIG, 0x10) != 0) goto fail;
    plat_sleep_ms(20);
    return 0;
fail:
    fprintf(stderr, "[imu] configuration failed: %s\n", strerror(errno));
    fault_set(FAULT_IMU_I2C);
    return -1;
}
void imu_close(void)
{
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
}
static void publish(void)
{
    shared_imu_put(&g_st);
    if (g_verbose_imu && (g_st.samples % 20u) == 0u)
        printf("[imu] %s ax=%+6.2f ay=%+6.2f az=%+6.2f m/s2  gx=%+6.1f gy=%+6.1f gz=%+6.1f dps  "
               "a_fwd=%+5.2f |a|=%.2fg%s err=%u\n",
               g_st.calibrated ? "CAL" : "---", g_st.ax, g_st.ay, g_st.az, g_st.gx, g_st.gy, g_st.gz,
               g_st.a_long, g_st.a_dyn_g, g_st.impact ? " IMPACT!" : "", (unsigned)g_st.i2c_errors);
}
void imu_ingest_phys(const float v[6], uint64_t t_ns)
{
    if (atomic_exchange(&g_clear_impact_req, false)) imu_proc_clear_impact(&g_ip);
    bool was_impact = g_st.impact;
    imu_proc_sample(&g_ip, v, t_ns, &g_params, &g_st);
    if (g_st.impact && !was_impact) {
        fault_set(FAULT_IMPACT);
        log_event(SEV_CRIT, "IMPACT detected: |a|=%.2f g (threshold %.1f g, %d samples)", g_st.a_dyn_g,
                  g_params.impact_threshold_g, g_params.impact_debounce_n);
    }
    publish();
}
void *imu_thread(void *arg)
{
    (void)arg;
    int chid, coid;
    timer_t tid;
    if (plat_channel_create(&chid, &coid) != 0 ||
        plat_timer_periodic(coid, PC_TIMER, PRIO_IMU, IMU_PERIOD_MS * 1000u, &tid) != 0) {
        log_event(SEV_CRIT, "imu thread: channel/timer creation failed (%s)", strerror(errno));
        fault_set(FAULT_IMU_I2C);
        return NULL;
    }
    int consec_err = 0, same_cnt = 0;
    uint8_t prev[14] = {0};
    bool cal_announced = false;
    int cal_attempts = 0;
    while (atomic_load(&g_running)) {
        struct _pulse p;
        if (MsgReceivePulse(chid, &p, sizeof p, NULL) == -1) continue;
        if (p.code != PC_TIMER) continue;
        hb_beat(HB_IMU);
        if (g_fd < 0) continue;
        uint8_t b[14];
        uint64_t t = plat_now_ns();
        if (i2c_read_regs(REG_ACCEL_XOUT_H, b, 14) != 0) {
            g_st.i2c_errors++;
            if (++consec_err == IMU_MAX_CONSEC_ERR) {
                g_st.valid = false;
                shared_imu_put(&g_st);
                fault_set(FAULT_IMU_I2C);
                log_event(SEV_ERROR, "IMU I2C read failing (%s) - IMU INVALID", strerror(errno));
            }
            continue;
        }
        if (consec_err >= IMU_MAX_CONSEC_ERR) log_event(SEV_WARN, "IMU I2C recovered");
        consec_err = 0;
        same_cnt = (memcmp(b, prev, 14) == 0) ? same_cnt + 1 : 0;
        memcpy(prev, b, 14);
        if (same_cnt == IMU_STUCK_SAMPLES) {
            g_st.valid = false;
            shared_imu_put(&g_st);
            fault_set(FAULT_IMU_IMPLAUSIBLE);
            log_event(SEV_ERROR, "IMU data frozen for %d samples - IMU INVALID", IMU_STUCK_SAMPLES);
        }
        if (same_cnt >= IMU_STUCK_SAMPLES) continue;
        int16_t raw[7];
        for (int i = 0; i < 7; i++) raw[i] = (int16_t)(((uint16_t)b[2 * i] << 8) | b[2 * i + 1]);
        float v[6] = {raw[0] / ACCEL_LSB_PER_G * G_MPS2, raw[1] / ACCEL_LSB_PER_G * G_MPS2,
                      raw[2] / ACCEL_LSB_PER_G * G_MPS2, raw[4] / GYRO_LSB_PER_DPS,
                      raw[5] / GYRO_LSB_PER_DPS,         raw[6] / GYRO_LSB_PER_DPS};
        if (!g_ip.calibrated) {
            if (!cal_announced) {
                log_event(SEV_INFO, "IMU calibrating (%d samples) - keep the car still", IMU_CAL_SAMPLES);
                cal_announced = true;
            }
            imu_proc_cal_add(&g_ip, v, IMU_CAL_MAX_GYRO_DPS);
            if (g_ip.n_cal >= IMU_CAL_SAMPLES) {
                if (imu_proc_cal_finish(&g_ip, IMU_CAL_SAMPLES)) {
                    log_event(SEV_INFO,
                              "IMU calibrated: gravity axis=%c bias a=(%.2f %.2f %.2f) g=(%.2f %.2f %.2f)",
                              "XYZ"[g_ip.g_axis], g_ip.bias[0], g_ip.bias[1], g_ip.bias[2], g_ip.bias[3],
                              g_ip.bias[4], g_ip.bias[5]);
                } else {
                    cal_attempts++;
                    log_event(SEV_WARN, "IMU calibration rejected (moving or not level) - retry %d",
                              cal_attempts);
                    imu_proc_init(&g_ip);
                }
            }
            continue;
        }
        imu_ingest_phys(v, t);
    }
    timer_delete(tid);
    plat_channel_destroy(chid, coid);
    return NULL;
}
int imu_i2c_scan(const char *dev)
{
    g_fd = open(dev, O_RDWR);
    if (g_fd < 0) {
        fprintf(stderr, "[scan] open %s: %s\n", dev, strerror(errno));
        return -1;
    }
    int found = 0;
    printf("[scan] scanning %s ...\n", dev);
    for (int a = 0x08; a <= 0x77; a++) {
        uint8_t v;
        g_addr = a;
        if (i2c_read_regs(0x75, &v, 1) == 0) {
            printf("[scan] device at 0x%02X  (reg 0x75 = 0x%02X%s)\n", a, v,
                   v == 0x68                                            ? " -> MPU6050"
                   : (v == 0x70 || v == 0x71 || v == 0x72 || v == 0x73) ? " -> MPU6500/9250 family"
                                                                        : "");
            found++;
        }
    }
    printf("[scan] %d device(s) found on %s\n", found, dev);
    close(g_fd);
    g_fd = -1;
    return found;
}
