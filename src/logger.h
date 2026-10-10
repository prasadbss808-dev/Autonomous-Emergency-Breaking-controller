#ifndef AEB_LOGGER_H
#define AEB_LOGGER_H
#include <stdbool.h>
#include "aeb_types.h"
enum { SEV_INFO = 0, SEV_WARN, SEV_ERROR, SEV_CRIT };
int log_init(const char *csv_path, bool console_status);
void log_event(int sev, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void log_cycle(const aeb_status_t *st, motor_cmd_t cmd, float cmd_latency_us);
void *logger_thread(void *arg);
void log_drain_now(void);
void log_close(void);
unsigned log_dropped(void);
#endif
