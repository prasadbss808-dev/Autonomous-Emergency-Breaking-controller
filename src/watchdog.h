#ifndef AEB_WATCHDOG_H
#define AEB_WATCHDOG_H
#include <stdint.h>
#include <stdbool.h>
int watchdog_init(bool require_guard);
void *watchdog_thread(void *arg);
void watchdog_arm_deadline(uint32_t seq);
void watchdog_stop(void);
bool watchdog_guard_connected(void);
void watchdog_guard_clean_exit(void);
#endif
