#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H
#include <stdint.h>
#include <stdbool.h>
int imu_init(const char *dev, int addr);
void *imu_thread(void *arg);
void imu_ingest_phys(const float v[6], uint64_t t_ns);
void imu_clear_impact(void);
void imu_set_verbose(bool v);
void imu_close(void);
int imu_i2c_scan(const char *dev);
#endif
