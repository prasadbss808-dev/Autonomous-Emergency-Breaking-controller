#include "config.h"
#include "params.h"
#include "shared_state.h"
#include "platform_qnx.h"
#include "logger.h"
#include "radar_rd03d.h"
#include "imu_mpu6050.h"
#include "motor_bts7960.h"
#include "brake.h"
#include "aeb_controller.h"
#include "watchdog.h"
#include "vehicle_state.h"
#include "camera_if.h"
#include "telemetry.h"
#include "sim_rt.h"
#include "sim_model.h"
#include "selftest.h"
#include "test_modes.h"
#include "aeb_cycle.h"
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#define AEB_VERSION "1.0"
static pthread_t t_logger, t_wdog, t_brake, t_motor, t_pred, t_veh, t_radar, t_imu, t_sim, t_sig;
static bool s_logger, s_wdog, s_brake, s_motor, s_pred, s_veh, s_radar, s_imu, s_sim, s_sig;
static void usage(void)
{
    printf("QNX AEB controller v%s\n"
           "usage: aeb [options]\n"
           "  -m MODE   full | sensor | radar | imu | motor | calib | sim | selftest | i2cscan\n"
           "  -s N      sim scenario 1..%d (sim mode)\n"
           "  -c FILE   parameter override file (key = value)\n"
           "  -l FILE   CSV log file (default %s, '-' = none)\n"
           "  -e SRC    ego speed source: none | radar_static | imu | encoder | sim\n"
           "  -p BE     motor backend: hw | soft | none\n"
           "  -u DEV    radar UART (default %s)\n"
           "  -i DEV    IMU I2C bus (default %s)\n"
           "  -o FILE   calibration output (calib mode, default aeb_calib.conf)\n"
           "  -G        refuse to drive without the aeb_guard process\n"
           "  -y        explicit enable for motor / calib modes\n"
           "  -q        no periodic console status line\n"
           "  -v        verbose (selftest: scenario timelines; radar: raw UART hex dump)\n"
           "  -P        print all parameters and exit\n",
           AEB_VERSION, sim_num_scenarios(), LOG_DEFAULT_PATH, RADAR_DEV, IMU_DEV);
}
static int parse_mode(const char *s, run_mode_t *m)
{
    static const struct {
        const char *n;
        run_mode_t m;
    } t[] = {{"full", MODE_FULL}, {"sensor", MODE_SENSOR},     {"radar", MODE_RADAR},
             {"imu", MODE_IMU},   {"motor", MODE_MOTOR},       {"calib", MODE_CALIB},
             {"sim", MODE_SIM},   {"selftest", MODE_SELFTEST}, {"i2cscan", MODE_I2CSCAN}};
    for (unsigned i = 0; i < sizeof t / sizeof t[0]; i++)
        if (strcmp(s, t[i].n) == 0) {
            *m = t[i].m;
            return 0;
        }
    return -1;
}
static void crash_handler(int sig)
{
    motor_signal_safe();
    static const char msg[] = "\n[aeb] FATAL signal - motor outputs forced OFF\n";
    ssize_t r = write(STDERR_FILENO, msg, sizeof msg - 1);
    (void)r;
    _exit(128 + sig);
}
static void *signal_thread(void *arg)
{
    sigset_t *set = (sigset_t *)arg;
    while (atomic_load(&g_running)) {
        siginfo_t si;
        int sig = sigwaitinfo(set, &si);
        if (sig == -1) continue;
        if (sig == SIGUSR1) continue;
        motor_force_safe(true, sig == SIGINT ? "Ctrl-C" : "termination signal");
        atomic_store(&g_cmd_stop, true);
        log_event(SEV_WARN, "signal %d received - shutting down safely", sig);
        atomic_store(&g_running, false);
    }
    return NULL;
}
static void print_status_detail(void)
{
    aeb_status_t s;
    shared_aeb_get(&s);
    radar_state_t r;
    shared_radar_get(&r);
    imu_state_t m;
    shared_imu_get(&m);
    vehicle_state_t v;
    shared_veh_get(&v);
    motor_cmd_t c = motor_get_applied();
    uint64_t now = plat_now_ns();
    printf("---------------------------------------------------------------\n");
    printf(" state %-16s risk %-8s cycle %u  faults 0x%04x  wdog-mask 0x%x\n", aeb_state_name(s.state),
           risk_name(s.risk), (unsigned)s.cycle, (unsigned)s.faults, atomic_load(&g_wdog_tripped_mask));
    printf(" radar %s age %.0f ms  frames %u bad %u rejected %u period %.1f ms tracks %u\n",
           s.radar_ok ? "OK " : "BAD", r.t_ns ? (double)age_ns(now, r.t_ns) / 1e6 : -1.0,
           (unsigned)r.frames_ok, (unsigned)r.frames_bad, (unsigned)r.targets_rejected, r.frame_period_ms,
           r.n_tracks);
    for (int i = 0; i < r.n_tracks; i++)
        printf("   track id%u x=%+.2f y=%.2f r=%.2f v_close=%+.2f %s %s\n", r.tracks[i].id, r.tracks[i].x_m,
               r.tracks[i].y_m, r.tracks[i].range_m, r.tracks[i].v_close_mps,
               r.tracks[i].confirmed ? "CONF" : "tent", r.tracks[i].in_corridor ? "IN-PATH" : "beside");
    printf(" threat %s r=%.2f vc=%.2f ttc=%.2f  d_safe P/S/F = %.2f / %.2f / %.2f m\n",
           s.threat.valid ? "YES" : "no ", s.threat.range_m, s.threat.v_close_mps,
           s.ttc_valid ? s.ttc_s : -1.0f, s.d_safe_m[0], s.d_safe_m[1], s.d_safe_m[2]);
    printf(" imu %s cal=%d a_fwd=%+.2f m/s2 |a|=%.2f g impact=%d i2c_err=%u\n", s.imu_ok ? "OK " : "BAD",
           m.calibrated, m.a_long, m.a_dyn_g, m.impact, (unsigned)m.i2c_errors);
    printf(" ego %s %.2f m/s (src %s) stopped=%d\n", v.ego_valid ? "valid" : "n/a  ", v.ego_speed_mps,
           ego_src_name(v.src), v.stopped);
    printf(" motor %s %s %.0f%% latched=%d guard=%s log-dropped=%u\n", motor_backend_name(motor_backend()),
           motor_cmd_name(c.type), c.duty_pct, motor_safe_latched(),
           watchdog_guard_connected() ? "connected" : "NOT connected", log_dropped());
    printf("---------------------------------------------------------------\n");
}
static void cli_handle(char *line)
{
    char *cmd = strtok(line, " \t\r\n");
    if (!cmd) return;
    if (!strcmp(cmd, "go") || !strcmp(cmd, "resume")) {
        atomic_store(&g_cmd_go, true);
        printf("[cli] go requested (accepted only if radar/IMU healthy and path clear)\n");
    } else if (!strcmp(cmd, "stop")) {
        atomic_store(&g_cmd_stop, true);
    } else if (!strcmp(cmd, "estop")) {
        motor_force_safe(true, "operator ESTOP");
        fault_set(FAULT_OPERATOR_ESTOP);
        atomic_store(&g_cmd_stop, true);
    } else if (!strcmp(cmd, "clear")) {
        imu_clear_impact();
        fault_clear(FAULT_IMPACT);
        printf("[cli] impact latch cleared\n");
    } else if (!strcmp(cmd, "status") || !strcmp(cmd, "s")) {
        print_status_detail();
    } else if (!strcmp(cmd, "params")) {
        params_print(&g_params);
    } else if (!strcmp(cmd, "quit") || !strcmp(cmd, "exit") || !strcmp(cmd, "q")) {
        atomic_store(&g_cmd_stop, true);
        atomic_store(&g_running, false);
    } else {
        printf("commands: go | resume | stop | estop | clear | status | params | quit\n");
    }
}
static int start(pthread_t *t, bool *flag, const char *name, void *(*fn)(void *), int policy, int prio,
                 unsigned mask, hb_id_t hb, int critical)
{
    if (critical >= 0) hb_enable(hb, critical != 0);
    int rc = plat_thread_start(t, name, fn, NULL, policy, prio, mask);
    if (rc != 0) {
        fprintf(stderr, "[aeb] cannot start thread %s: %s (run as root?)\n", name, strerror(rc));
        return -1;
    }
    *flag = true;
    return 0;
}
int main(int argc, char **argv)
{
    const char *conf = NULL, *log_path = LOG_DEFAULT_PATH, *radar_dev = RADAR_DEV;
    const char *imu_dev = IMU_DEV, *calib_out = "aeb_calib.conf";
    const char *mode_name = "full";
    int scenario = 5, opt;
    bool require_guard = false, yes = false, quiet = false, print_params = false;
    int backend_opt = -1;
    bool ego_given = false;
    while ((opt = getopt(argc, argv, "m:s:c:l:e:p:u:i:o:GyqvPh")) != -1) {
        switch (opt) {
        case 'm':
            if (parse_mode(optarg, &g_mode)) {
                usage();
                return 2;
            }
            mode_name = optarg;
            break;
        case 's': scenario = atoi(optarg); break;
        case 'c': conf = optarg; break;
        case 'l': log_path = strcmp(optarg, "-") ? optarg : NULL; break;
        case 'e':
            if (ego_src_parse(optarg, &g_ego_src)) {
                usage();
                return 2;
            }
            ego_given = true;
            break;
        case 'p':
            backend_opt = !strcmp(optarg, "hw")     ? MOTOR_BACKEND_HWPWM
                          : !strcmp(optarg, "soft") ? MOTOR_BACKEND_SOFTPWM
                          : !strcmp(optarg, "none") ? MOTOR_BACKEND_NONE
                                                    : -2;
            if (backend_opt == -2) {
                usage();
                return 2;
            }
            break;
        case 'u': radar_dev = optarg; break;
        case 'i': imu_dev = optarg; break;
        case 'o': calib_out = optarg; break;
        case 'G': require_guard = true; break;
        case 'y': yes = true; break;
        case 'q': quiet = true; break;
        case 'v': g_verbose = 1; break;
        case 'P': print_params = true; break;
        default: usage(); return 2;
        }
    }
    params_set_defaults(&g_params);
    if (conf) {
        int n = params_load_file(&g_params, conf);
        if (n < 0) {
            fprintf(stderr, "[aeb] cannot read %s: %s\n", conf, strerror(errno));
            return 2;
        }
        printf("[aeb] %d parameters loaded from %s\n", n, conf);
    }
    if (print_params) {
        params_print(&g_params);
        return 0;
    }
    if (params_validate(&g_params) != 0) {
        fprintf(stderr, "[aeb] invalid parameters - refusing to start\n");
        return 2;
    }
    if (g_mode == MODE_I2CSCAN) return imu_i2c_scan(imu_dev) > 0 ? 0 : 1;
    if (g_mode == MODE_SELFTEST) {
        int f = selftest_run_all(g_verbose != 0);
        return f ? 1 : 0;
    }
    if ((g_mode == MODE_MOTOR || g_mode == MODE_CALIB) && !yes) {
        fprintf(stderr, "[aeb] motor/calib modes move the vehicle: add -y to confirm\n");
        return 2;
    }
    if (g_mode == MODE_SIM) {
        if (sim_rt_init(scenario) != 0) {
            fprintf(stderr, "[aeb] unknown scenario %d\n", scenario);
            return 2;
        }
        if (!ego_given) g_ego_src = EGO_SRC_SIM;
        params_apply_test_profile(&g_params);
    }
    if (plat_lock_memory() != 0)
        fprintf(stderr, "[aeb] warning: mlockall failed (%s) - page faults possible\n", strerror(errno));
    struct sched_param sp = {.sched_priority = PRIO_LOGGER};
    pthread_setschedparam(pthread_self(), SCHED_RR, &sp);
    printf("[aeb] QNX AEB v%s  mode=%d  cpus=%d  pid=%d\n", AEB_VERSION, (int)g_mode, plat_num_cpus(),
           (int)getpid());
    if (plat_num_cpus() < 4) printf("[aeb] note: fewer than 4 CPUs - runmasks partially ignored\n");
    static sigset_t sigs;
    sigemptyset(&sigs);
    sigaddset(&sigs, SIGINT);
    sigaddset(&sigs, SIGTERM);
    sigaddset(&sigs, SIGHUP);
    sigaddset(&sigs, SIGUSR1);
    pthread_sigmask(SIG_BLOCK, &sigs, NULL);
    signal(SIGPIPE, SIG_IGN);
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = crash_handler;
    sigaction(SIGSEGV, &sa, NULL);
    sigaction(SIGBUS, &sa, NULL);
    sigaction(SIGILL, &sa, NULL);
    sigaction(SIGFPE, &sa, NULL);
    sigaction(SIGABRT, &sa, NULL);
    shared_init();
    if (log_init(log_path, !quiet && (g_mode == MODE_FULL || g_mode == MODE_SENSOR || g_mode == MODE_SIM)) !=
        0)
        fprintf(stderr, "[aeb] warning: CSV log disabled\n");
    if (log_path) printf("[aeb] CSV log: %s\n", log_path);
    motor_backend_t be;
    if (backend_opt >= 0)
        be = (motor_backend_t)backend_opt;
    else
        be = (g_mode == MODE_FULL || g_mode == MODE_MOTOR || g_mode == MODE_CALIB) ? MOTOR_BACKEND_HWPWM
                                                                                   : MOTOR_BACKEND_NONE;
    if (be == MOTOR_BACKEND_NONE) {
        if (motor_park_pins() == 0)
            printf("[aeb] motor pins parked LOW (drivers OFF), motors not used\n");
        else
            printf("[aeb] rpi_gpio not reachable - motors not used (dry run)\n");
    } else if (motor_init(be) != 0) {
        fprintf(stderr, "[aeb] motor init failed - refusing to run a motor mode\n");
        return 1;
    }
    printf("[aeb] motor backend: %s\n", motor_backend_name(motor_backend()));
    printf("[aeb] motor pins: front RPWM %d LPWM %d EN %d | rear RPWM %d LPWM %d EN %d\n",
           g_params.pin_front_rpwm, g_params.pin_front_lpwm, g_params.pin_front_en, g_params.pin_rear_rpwm,
           g_params.pin_rear_lpwm, g_params.pin_rear_en);
    if (g_params.motor_axles != 3)
        printf("[aeb] motor axles: %s ONLY - the %s driver is kept OFF (motor_axles = %d)\n",
               g_params.motor_axles == 2 ? "REAR" : "FRONT", g_params.motor_axles == 2 ? "front" : "rear",
               g_params.motor_axles);
    atexit(motor_shutdown);
    if (brake_init() != 0) {
        perror("[aeb] brake channel");
        return 1;
    }
    int wr = watchdog_init(require_guard);
    if (wr == -1) {
        perror("[aeb] watchdog channel/timer");
        return 1;
    }
    if (wr == -2) {
        fprintf(stderr, "[aeb] aeb_guard not running and -G given - refusing to start\n");
        return 1;
    }
    printf("[aeb] external guard: %s\n",
           watchdog_guard_connected() ? "connected"
                                      : "NOT running (start 'aeb_guard &' first for crash protection)");
    camera_init();
    if (telemetry_init(mode_name) == 0)
        printf("[aeb] dashboard telemetry: %s (start ./aeb_dash, open http://<pi-ip>:8080)\n",
               TELEM_SHM_NAME);
    bool need_radar =
        (g_mode == MODE_FULL || g_mode == MODE_SENSOR || g_mode == MODE_RADAR || g_mode == MODE_CALIB);
    bool need_imu =
        (g_mode == MODE_FULL || g_mode == MODE_SENSOR || g_mode == MODE_IMU || g_mode == MODE_CALIB);
    if (g_mode == MODE_SIM) {
        radar_init(NULL, 0);
        imu_init(NULL, 0);
    }
    if (need_radar) {
        if (radar_init(radar_dev, RADAR_BAUD) == 0)
            printf("[aeb] radar %s @ %d 8N1 open\n", radar_dev, RADAR_BAUD);
        else
            printf("[aeb] radar NOT available - driving will be refused (thread keeps retrying)\n");
        radar_set_verbose(g_mode == MODE_RADAR);
        radar_set_hexdump(g_mode == MODE_RADAR && g_verbose);
    }
    if (need_imu) {
        if (imu_init(imu_dev, IMU_ADDR) == 0)
            printf("[aeb] MPU6050 %s addr 0x%02X configured\n", imu_dev, IMU_ADDR);
        else
            printf("[aeb] IMU NOT available\n");
        imu_set_verbose(g_mode == MODE_IMU);
    }
    int err = 0;
    err |= plat_thread_start(&t_sig, "signals", signal_thread, &sigs, SCHED_RR, PRIO_LOGGER, 0) ? -1 : 0;
    s_sig = (err == 0);
    err |= start(&t_logger, &s_logger, "logger", logger_thread, SCHED_RR, PRIO_LOGGER, CPU_MASK_SENSORS,
                 HB_LOGGER, 0);
    err |= start(&t_wdog, &s_wdog, "watchdog", watchdog_thread, SCHED_FIFO, PRIO_WATCHDOG, CPU_MASK_WATCHDOG,
                 HB_WATCHDOG, -1);
    bool aeb_pipeline = (g_mode == MODE_FULL || g_mode == MODE_SENSOR || g_mode == MODE_SIM);
    if (aeb_pipeline || g_mode == MODE_MOTOR || g_mode == MODE_CALIB) {
        err |= start(&t_brake, &s_brake, "brake", brake_thread, SCHED_FIFO, PRIO_BRAKE, CPU_MASK_BRAKE,
                     HB_BRAKE, 1);
        err |= start(&t_motor, &s_motor, "motor_pwm", motor_thread, SCHED_FIFO, PRIO_PWM, CPU_MASK_PWM,
                     HB_MOTOR, 1);
    }
    if (need_radar)
        err |= start(&t_radar, &s_radar, "radar", radar_thread, SCHED_FIFO, PRIO_RADAR, CPU_MASK_SENSORS,
                     HB_RADAR, 1);
    if (need_imu)
        err |= start(&t_imu, &s_imu, "imu", imu_thread, SCHED_FIFO, PRIO_IMU, CPU_MASK_SENSORS, HB_IMU, 1);
    if (g_mode == MODE_SIM)
        err |= start(&t_sim, &s_sim, "sim", sim_thread, SCHED_FIFO, PRIO_SIM, CPU_MASK_SENSORS, HB_SIM, 1);
    if (aeb_pipeline) {
        err |= start(&t_veh, &s_veh, "vehicle", vehicle_thread, SCHED_RR, PRIO_VEHICLE, CPU_MASK_SENSORS,
                     HB_VEHICLE, 0);
        err |= start(&t_pred, &s_pred, "prediction", prediction_thread, SCHED_FIFO, PRIO_PREDICTION,
                     CPU_MASK_PREDICTION, HB_PREDICT, 1);
    }
    if (err) {
        motor_force_safe(false, "thread start failure");
        atomic_store(&g_running, false);
    }
    int exit_code = 0;
    if (atomic_load(&g_running)) {
        if (g_mode == MODE_MOTOR) {
            exit_code = motor_test_run();
            atomic_store(&g_running, false);
        } else if (g_mode == MODE_CALIB) {
            exit_code = calib_run(calib_out);
            atomic_store(&g_running, false);
        } else {
            if (g_mode == MODE_FULL)
                printf(
                    "\n[aeb] READY. %s\n"
                    "      Any object in the path closer than %.2f m -> IMMEDIATE STOP (then type 'go' to restart).\n"
                    "      Commands: go | stop | estop | status | quit.  Master switch = hardware stop.\n\n",
                    g_params.auto_start
                        ? "Vehicle STARTS AUTOMATICALLY in ~2 s once the radar is healthy and the path is clear."
                        : "Type 'go' to drive.",
                    g_params.proximity_stop_m);
            else if (g_mode == MODE_SENSOR)
                printf("\n[aeb] SENSOR TEST: full AEB logic, motors NOT driven. Driving starts %s;\n"
                       "      walk toward the radar and watch the states (stop distance %.2f m).\n\n",
                       g_params.auto_start ? "automatically" : "on 'go'", g_params.proximity_stop_m);
            struct pollfd pfd = {STDIN_FILENO, POLLIN, 0};
            char line[128];
            while (atomic_load(&g_running)) {
                if (g_mode == MODE_SIM && atomic_load(&g_sim_result) >= 0) {
                    plat_sleep_ms(300);
                    exit_code = atomic_load(&g_sim_result);
                    break;
                }
                switch (control_poll()) {
                case CTL_GO: {
                    char c[] = "go";
                    printf("[dash] remote GO\n");
                    cli_handle(c);
                    break;
                }
                case CTL_STOP: {
                    char c[] = "stop";
                    printf("[dash] remote STOP\n");
                    cli_handle(c);
                    break;
                }
                case CTL_ESTOP: {
                    char c[] = "estop";
                    printf("[dash] remote E-STOP\n");
                    cli_handle(c);
                    break;
                }
                case CTL_CLEAR: {
                    char c[] = "clear";
                    printf("[dash] remote CLEAR\n");
                    cli_handle(c);
                    break;
                }
                default: break;
                }
                int r = poll(&pfd, 1, 100);
                if (r > 0 && (pfd.revents & (POLLIN | POLLHUP))) {
                    if (!fgets(line, sizeof line, stdin))
                        pfd.fd = -1;
                    else
                        cli_handle(line);
                }
            }
        }
    }
    atomic_store(&g_running, false);
    motor_cmd_t applied = motor_get_applied();
    if (applied.type == MOTOR_DRIVE) {
        motor_force_safe(true, "shutdown");
        plat_sleep_ms(500);
    }
    pthread_t *first[] = {&t_pred, &t_veh, &t_sim, &t_radar, &t_imu};
    bool *ff[] = {&s_pred, &s_veh, &s_sim, &s_radar, &s_imu};
    for (unsigned i = 0; i < sizeof first / sizeof first[0]; i++)
        if (*ff[i]) pthread_join(*first[i], NULL);
    brake_stop();
    if (s_brake) pthread_join(t_brake, NULL);
    if (s_motor) pthread_join(t_motor, NULL);
    watchdog_stop();
    if (s_wdog) pthread_join(t_wdog, NULL);
    if (s_logger) pthread_join(t_logger, NULL);
    if (s_sig) {
        pthread_kill(t_sig, SIGUSR1);
        pthread_join(t_sig, NULL);
    }
    motor_shutdown();
    watchdog_guard_clean_exit();
    log_close();
    radar_close();
    imu_close();
    printf("[aeb] stopped. motor outputs DISABLED. faults=0x%04x\n", (unsigned)fault_get());
    return exit_code;
}
