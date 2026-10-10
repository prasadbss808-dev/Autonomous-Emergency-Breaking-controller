#include "../src/config.h"
#include "../src/params.h"
#include "../src/gpio_rpi.h"
#include "../src/ipc.h"
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
typedef union {
    struct _pulse pulse;
    uint16_t type;
    guard_msg_t msg;
} guard_rx_t;
static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}
static int force_outputs(bool brake)
{
    int e = 0;
    const int rp[2] = {g_params.pin_front_rpwm, g_params.pin_rear_rpwm};
    const int lp[2] = {g_params.pin_front_lpwm, g_params.pin_rear_lpwm};
    const int en[2] = {g_params.pin_front_en, g_params.pin_rear_en};
    for (int i = 0; i < 2; i++) {
        gpio_pwm_duty(rp[i], 0.0f);
        if (gpio_write(rp[i], 0) != 0) e++;
        if (gpio_write(lp[i], 0) != 0) e++;
    }
    for (int i = 0; i < 2; i++)
        if (gpio_write(en[i], (brake && e == 0) ? 1 : 0) != 0) e++;
    return e;
}
int main(int argc, char **argv)
{
    params_set_defaults(&g_params);
    if (argc > 2 && strcmp(argv[1], "-c") == 0) {
        if (params_load_file(&g_params, argv[2]) < 0) {
            perror("aeb_guard: config");
            return 1;
        }
    }
    if (gpio_open(GPIO_MSG_PATH) != 0) {
        fprintf(stderr, "aeb_guard: cannot open %s: %s\n", GPIO_MSG_PATH, strerror(errno));
        return 1;
    }
    name_attach_t *att = name_attach(NULL, GUARD_NAME, 0);
    if (!att) {
        perror("aeb_guard: name_attach");
        return 1;
    }
    int self = ConnectAttach(0, 0, att->chid, _NTO_SIDE_CHANNEL, 0);
    if (self == -1) {
        perror("aeb_guard: ConnectAttach");
        return 1;
    }
    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, self, PRIO_GUARD, PC_GUARD_TICK, 0);
    timer_t tid;
    struct itimerspec its = {{0, 20000000L}, {0, 20000000L}};
    if (timer_create(CLOCK_MONOTONIC, &ev, &tid) == -1 || timer_settime(tid, 0, &its, NULL) == -1) {
        perror("aeb_guard: timer");
        return 1;
    }
    struct sched_param sp = {.sched_priority = PRIO_GUARD};
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    printf("aeb_guard: running (prio %d, timeout %d ms), waiting for AEB process\n", PRIO_GUARD,
           GUARD_TIMEOUT_MS);
    bool armed = false, braking = false;
    pid_t client = -1;
    uint64_t last_hb = 0, t_trip = 0;
    for (;;) {
        guard_rx_t rx;
        struct _msg_info info;
        rcvid_t rcvid = MsgReceive(att->chid, &rx, sizeof rx, &info);
        if (rcvid == -1) continue;
        uint64_t now = now_ns();
        if (rcvid == 0) {
            switch (rx.pulse.code) {
            case PC_GUARD_HB:
                last_hb = now;
                armed = true;
                break;
            case PC_GUARD_TICK:
                if (armed && now - last_hb > (uint64_t)GUARD_TIMEOUT_MS * 1000000ull) {
                    int e = force_outputs(true);
                    fprintf(stderr, "aeb_guard: HEARTBEAT TIMEOUT (%.0f ms) -> outputs SAFE%s\n",
                            (double)(now - last_hb) / 1e6, e ? " (GPIO errors, EN low)" : "");
                    if (GUARD_KILL_ON_TIMEOUT && client > 0) {
                        kill(client, SIGKILL);
                        fprintf(stderr, "aeb_guard: AEB process %d killed\n", (int)client);
                    }
                    armed = false;
                    braking = true;
                    t_trip = now;
                }
                if (braking && now - t_trip > (uint64_t)GUARD_BRAKE_HOLD_MS * 1000000ull) {
                    force_outputs(false);
                    braking = false;
                    fprintf(stderr, "aeb_guard: brake hold done -> drivers DISABLED\n");
                }
                break;
            case _PULSE_CODE_DISCONNECT:
                ConnectDetach(rx.pulse.scoid);
                if (armed) {
                    int e = force_outputs(true);
                    fprintf(stderr, "aeb_guard: AEB process DISCONNECTED (crash/exit) -> outputs SAFE%s\n",
                            e ? " (GPIO errors)" : "");
                    braking = true;
                    t_trip = now;
                } else {
                    force_outputs(false);
                }
                armed = false;
                client = -1;
                break;
            case _PULSE_CODE_UNBLOCK:
            default: break;
            }
            continue;
        }
        if (rx.type == _IO_CONNECT) {
            MsgReply(rcvid, EOK, NULL, 0);
            continue;
        }
        if (rx.type > _IO_BASE && rx.type <= _IO_MAX) {
            MsgError(rcvid, ENOSYS);
            continue;
        }
        if (rx.type == MSG_GUARD_REGISTER) {
            client = info.pid;
            last_hb = now;
            armed = false;
            printf("aeb_guard: AEB process %d registered\n", (int)client);
            MsgReply(rcvid, EOK, NULL, 0);
        } else if (rx.type == MSG_GUARD_CLEAN_EXIT) {
            armed = false;
            force_outputs(false);
            printf("aeb_guard: AEB process %d exited cleanly, outputs disabled\n", (int)info.pid);
            client = -1;
            MsgReply(rcvid, EOK, NULL, 0);
        } else {
            MsgError(rcvid, ENOSYS);
        }
    }
    return 0;
}
