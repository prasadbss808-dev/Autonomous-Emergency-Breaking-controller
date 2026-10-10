#ifndef MOTOR_BTS7960_H
#define MOTOR_BTS7960_H
#include <stdbool.h>
#include <stdint.h>
#include "aeb_types.h"
typedef enum { MOTOR_BACKEND_NONE = 0, MOTOR_BACKEND_HWPWM, MOTOR_BACKEND_SOFTPWM } motor_backend_t;
enum { DRV_FRONT = 1, DRV_REAR = 2, DRV_BOTH = 3 };
int motor_init(motor_backend_t requested);
motor_backend_t motor_backend(void);
const char *motor_backend_name(motor_backend_t b);
int motor_apply(motor_cmd_t cmd);
int motor_test_driver(int drv_mask, float duty_pct);
void motor_force_safe(bool brake, const char *why);
bool motor_safe_latched(void);
motor_cmd_t motor_get_applied(void);
uint64_t motor_last_apply_ns(void);
void *motor_thread(void *arg);
void motor_shutdown(void);
int motor_park_pins(void);
void motor_signal_safe(void);
#endif
