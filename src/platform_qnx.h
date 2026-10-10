#ifndef PLATFORM_QNX_H
#define PLATFORM_QNX_H
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>
uint64_t plat_now_ns(void);
void plat_sleep_ms(unsigned ms);
void plat_sleep_until_ns(uint64_t abs_ns);
int plat_num_cpus(void);
int plat_thread_start(pthread_t *tid, const char *name, void *(*fn)(void *), void *arg, int policy, int prio,
                      unsigned runmask);
int plat_channel_create(int *chid, int *coid);
void plat_channel_destroy(int chid, int coid);
int plat_timer_periodic(int coid, int code, int prio, uint32_t period_us, timer_t *tid);
int plat_timer_oneshot(int coid, int code, int prio, timer_t *tid);
int plat_timer_arm_us(timer_t tid, uint32_t us);
int plat_lock_memory(void);
int plat_set_runmask(unsigned mask);
#endif
