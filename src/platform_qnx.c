#include "platform_qnx.h"
#include <errno.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdatomic.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/siginfo.h>
#ifdef __QNX__
#include <sys/syspage.h>
#endif
uint64_t plat_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}
void plat_sleep_until_ns(uint64_t abs_ns)
{
    struct timespec ts = {(time_t)(abs_ns / 1000000000ull), (long)(abs_ns % 1000000000ull)};
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR) {
    }
}
void plat_sleep_ms(unsigned ms)
{
    plat_sleep_until_ns(plat_now_ns() + (uint64_t)ms * 1000000ull);
}
int plat_num_cpus(void)
{
#ifdef __QNX__
    return _syspage_ptr->num_cpu;
#else
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    return n > 0 ? (int)n : 1;
#endif
}
int plat_set_runmask(unsigned mask)
{
    if (mask == 0) return 0;
    unsigned ncpu = (unsigned)plat_num_cpus();
    unsigned all = (ncpu >= 32) ? 0xFFFFFFFFu : ((1u << ncpu) - 1u);
    if ((mask & all) == 0) return -1;
    if (ThreadCtl(_NTO_TCTL_RUNMASK, (void *)(uintptr_t)(mask & all)) == -1) return -1;
    return 0;
}
typedef struct {
    void *(*fn)(void *);
    void *arg;
    unsigned runmask;
    char name[24];
} tramp_t;
#define MAX_THREADS 24
static tramp_t g_tramp[MAX_THREADS];
static atomic_int g_tramp_n;
static void *trampoline(void *a)
{
    tramp_t *t = (tramp_t *)a;
    if (plat_set_runmask(t->runmask) != 0)
        fprintf(stderr, "[plat] warning: runmask 0x%x not applied for %s (errno %d)\n", t->runmask, t->name,
                errno);
    return t->fn(t->arg);
}
int plat_thread_start(pthread_t *tid, const char *name, void *(*fn)(void *), void *arg, int policy, int prio,
                      unsigned runmask)
{
    int idx = atomic_fetch_add(&g_tramp_n, 1);
    if (idx >= MAX_THREADS) return EAGAIN;
    tramp_t *t = &g_tramp[idx];
    t->fn = fn;
    t->arg = arg;
    t->runmask = runmask;
    strncpy(t->name, name, sizeof t->name - 1);
    pthread_attr_t at;
    struct sched_param sp;
    int rc = pthread_attr_init(&at);
    if (rc) return rc;
    pthread_attr_setinheritsched(&at, PTHREAD_EXPLICIT_SCHED);
    if ((rc = pthread_attr_setschedpolicy(&at, policy)) != 0) goto out;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = prio;
    if ((rc = pthread_attr_setschedparam(&at, &sp)) != 0) goto out;
    pthread_attr_setstacksize(&at, 256 * 1024);
    if ((rc = pthread_create(tid, &at, trampoline, t)) != 0) goto out;
    pthread_setname_np(*tid, name);
out:
    pthread_attr_destroy(&at);
    return rc;
}
int plat_channel_create(int *chid, int *coid)
{
    unsigned flags = 0;
#ifdef _NTO_CHF_PRIVATE
    flags |= _NTO_CHF_PRIVATE;
#endif
    *chid = ChannelCreate(flags);
    if (*chid == -1) return -1;
    *coid = ConnectAttach(0, 0, *chid, _NTO_SIDE_CHANNEL, 0);
    if (*coid == -1) {
        ChannelDestroy(*chid);
        *chid = -1;
        return -1;
    }
    return 0;
}
void plat_channel_destroy(int chid, int coid)
{
    if (coid >= 0) ConnectDetach(coid);
    if (chid >= 0) ChannelDestroy(chid);
}
static int timer_make(int coid, int code, int prio, timer_t *tid)
{
    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, coid, prio, code, 0);
    return (timer_create(CLOCK_MONOTONIC, &ev, tid) == -1) ? -1 : 0;
}
int plat_timer_periodic(int coid, int code, int prio, uint32_t period_us, timer_t *tid)
{
    if (timer_make(coid, code, prio, tid) != 0) return -1;
    struct itimerspec its;
    memset(&its, 0, sizeof its);
    its.it_value.tv_sec = period_us / 1000000u;
    its.it_value.tv_nsec = (long)(period_us % 1000000u) * 1000L;
    its.it_interval = its.it_value;
    if (timer_settime(*tid, 0, &its, NULL) == -1) {
        timer_delete(*tid);
        return -1;
    }
    return 0;
}
int plat_timer_oneshot(int coid, int code, int prio, timer_t *tid)
{
    return timer_make(coid, code, prio, tid);
}
int plat_timer_arm_us(timer_t tid, uint32_t us)
{
    struct itimerspec its;
    memset(&its, 0, sizeof its);
    its.it_value.tv_sec = us / 1000000u;
    its.it_value.tv_nsec = (long)(us % 1000000u) * 1000L;
    return (timer_settime(tid, 0, &its, NULL) == -1) ? -1 : 0;
}
int plat_lock_memory(void)
{
    return mlockall(MCL_CURRENT | MCL_FUTURE);
}
