#include "test_modes.h"
#include "motor_bts7960.h"
#include "aeb_logic.h"
#include "aeb_cycle.h"
#include "shared_state.h"
#include "platform_qnx.h"
#include "logger.h"
#include "params.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
static float test_duty(void)
{
    float d = 25.0f;
    if (d > MOTOR_TEST_MAX_DUTY) d = MOTOR_TEST_MAX_DUTY;
    if (d > g_params.motor_max_duty) d = g_params.motor_max_duty;
    return d;
}
static bool wait_ms(unsigned ms)
{
    uint64_t end = plat_now_ns() + (uint64_t)ms * 1000000ull;
    while (plat_now_ns() < end) {
        if (!atomic_load(&g_running) || motor_safe_latched()) return false;
        plat_sleep_ms(10);
    }
    return true;
}
static bool step(const char *what, int mask, float duty, unsigned ms)
{
    if (ms > MOTOR_TEST_MAX_MS) ms = MOTOR_TEST_MAX_MS;
    printf("\n>>> %s  (duty %.0f%%, %u ms)\n", what, duty, ms);
    fflush(stdout);
    if (motor_test_driver(mask, duty) != 0) {
        printf("    GPIO ERROR\n");
        return false;
    }
    bool ok = wait_ms(ms);
    motor_test_driver(DRV_BOTH, 0.0f);
    ok = wait_ms(500) && ok;
    motor_cmd_t off = {MOTOR_DISABLE, 0};
    motor_apply(off);
    return wait_ms(800) && ok;
}
int motor_test_run(void)
{
    float d = test_duty();
    printf("\n================ MOTOR TEST ================\n"
           "Backend: %s\n"
           "!! Put the car on a stand (wheels OFF the ground) !!\n"
           "Master switch ON. Ctrl-C = immediate safe stop.\n",
           motor_backend_name(motor_backend()));
    if (!wait_ms(3000)) return 1;
    bool ok = true;
    ok = ok && step("FRONT driver (BTS7960 #1): M1 front-left AND M3 front-right must BOTH turn FORWARD",
                    DRV_FRONT, d, 1000);
    ok = ok && step("REAR driver (BTS7960 #2): M2 rear-left AND M4 rear-right must BOTH turn FORWARD",
                    DRV_REAR, d, 1000);
    ok = ok && step("BOTH drivers: all four wheels FORWARD, then SHORT-BRAKE", DRV_BOTH, d, 1000);
    if (ok) {
        printf("\n>>> BOTH forward then COAST (EN low) - compare run-down with the brake step above\n");
        motor_test_driver(DRV_BOTH, d);
        ok = wait_ms(1000);
        motor_cmd_t off = {MOTOR_DISABLE, 0};
        motor_apply(off);
        ok = wait_ms(1500) && ok;
    }
    motor_cmd_t off = {MOTOR_DISABLE, 0};
    motor_apply(off);
    printf("\nMOTOR TEST %s. If ONE wheel of an axle turned BACKWARD, swap only THAT motor's\n"
           "two wires at the driver's M+/M- terminals. If BOTH wheels of an axle turned backward,\n"
           "swap the driver's M+ and M- output wires.\n",
           ok ? "COMPLETE" : "ABORTED");
    return ok ? 0 : 1;
}
static bool path_clear(float min_m)
{
    radar_state_t rs;
    shared_radar_get(&rs);
    if (!radar_is_fresh(&rs, plat_now_ns(), &g_params)) return true;
    for (int i = 0; i < rs.n_tracks; i++)
        if (rs.tracks[i].confirmed && rs.tracks[i].in_corridor && rs.tracks[i].range_m < min_m) return false;
    return true;
}
static int read_line(char *buf, int n)
{
    if (!fgets(buf, n, stdin)) return -1;
    return 0;
}
int calib_run(const char *out_path)
{
    const char *names[4] = {"PARTIAL", "STRONG", "FULL(short-brake)", "COAST(reference)"};
    float mean_decel[4] = {0}, peak_decel[4] = {0};
    bool done[4] = {false};
    const unsigned accel_ms = 1200;
    printf("\n========== BRAKING DECELERATION CALIBRATION ==========\n"
           "Needs: IMU calibrated, ~3 m of free straight floor per run, master switch ON.\n"
           "Each run: drive at cruise duty %.0f%% for %u ms, then apply the level and\n"
           "measure deceleration with the MPU6050 for 1.5 s.\n",
           g_params.cruise_duty, accel_ms);
    for (int lvl = 0; lvl < 4 && atomic_load(&g_running); lvl++) {
        imu_state_t im;
        shared_imu_get(&im);
        if (!imu_is_fresh(&im, plat_now_ns(), &g_params)) {
            printf("IMU not ready (calibrating or failed) - wait and retry.\n");
            plat_sleep_ms(2000);
            lvl--;
            continue;
        }
        printf("\nPlace the car with free space ahead. ENTER = run '%s', s = skip, q = quit: ", names[lvl]);
        fflush(stdout);
        char line[32];
        if (read_line(line, sizeof line) != 0 || line[0] == 'q') break;
        if (line[0] == 's') continue;
        if (!path_clear(2.0f)) {
            printf("Radar sees an in-path object closer than 2 m - aborted.\n");
            lvl--;
            continue;
        }
        motor_cmd_t drive = {MOTOR_DRIVE, g_params.cruise_duty};
        motor_apply(drive);
        uint64_t t_end = plat_now_ns() + (uint64_t)accel_ms * 1000000ull;
        bool abort = false;
        while (plat_now_ns() < t_end) {
            if (!atomic_load(&g_running) || motor_safe_latched() || !path_clear(1.0f)) {
                abort = true;
                break;
            }
            plat_sleep_ms(10);
        }
        motor_cmd_t c;
        switch (lvl) {
        case 0: c = (motor_cmd_t){MOTOR_DRIVE, g_params.partial_duty}; break;
        case 1:
            c = g_params.strong_duty > 0 ? (motor_cmd_t){MOTOR_DRIVE, g_params.strong_duty}
                                         : (motor_cmd_t){MOTOR_BRAKE, 0};
            break;
        case 2: c = (motor_cmd_t){MOTOR_BRAKE, 0}; break;
        default: c = (motor_cmd_t){MOTOR_DISABLE, 0}; break;
        }
        if (abort) c = (motor_cmd_t){MOTOR_BRAKE, 0};
        uint64_t t0 = plat_now_ns();
        motor_apply(c);
        double sum = 0;
        int n = 0;
        float peak = 0;
        uint64_t last_t = 0;
        while (plat_now_ns() - t0 < 1500000000ull) {
            shared_imu_get(&im);
            if (im.t_ns != last_t && im.t_ns > t0 + (uint64_t)g_params.verify_delay_ms * 1000000ull) {
                last_t = im.t_ns;
                float dcl = -im.a_long;
                if (dcl > peak) peak = dcl;
                if (dcl > 0.15f) {
                    sum += dcl;
                    n++;
                }
            }
            plat_sleep_ms(5);
        }
        motor_cmd_t brake = {MOTOR_BRAKE, 0};
        motor_apply(brake);
        plat_sleep_ms(500);
        motor_cmd_t off = {MOTOR_DISABLE, 0};
        motor_apply(off);
        if (abort) {
            printf("Run aborted (obstacle / stop).\n");
            break;
        }
        mean_decel[lvl] = n ? (float)(sum / n) : 0.0f;
        peak_decel[lvl] = peak;
        done[lvl] = true;
        printf("  %-18s mean decel %.2f m/s^2  peak %.2f m/s^2  (%d samples)\n", names[lvl], mean_decel[lvl],
               peak_decel[lvl], n);
    }
    printf("\n================ RESULT ================\n");
    for (int i = 0; i < 4; i++)
        printf("  %-18s %s mean %.2f  peak %.2f m/s^2\n", names[i], done[i] ? "    " : "SKIP", mean_decel[i],
               peak_decel[i]);
    FILE *f = fopen(out_path, "w");
    if (!f) {
        perror("calib: fopen");
        return 1;
    }
    fprintf(f, "# AEB braking calibration (measured with MPU6050). Load with: aeb -c %s\n", out_path);
    fprintf(f, "# model values: partial %.2f strong %.2f full %.2f m/s^2\n", g_params.decel_model[0],
            g_params.decel_model[1], g_params.decel_model[2]);
    const char *keys[3] = {"decel_measured_partial", "decel_measured_strong", "decel_measured_full"};
    for (int i = 0; i < 3; i++) {
        if (done[i] && mean_decel[i] > 0.05f)
            fprintf(f, "%s = %.3f\n", keys[i], mean_decel[i]);
        else
            fprintf(f, "# %s = (not measured)\n", keys[i]);
    }
    if (done[3]) fprintf(f, "# coast reference decel = %.3f\n", mean_decel[3]);
    fclose(f);
    printf("Written %s. Use the measured values: aeb -c %s\n", out_path, out_path);
    return 0;
}
