#include "selftest.h"
#include "aeb_cycle.h"
#include "imu_proc.h"
#include "sim_model.h"
#include <stdio.h>
#include <string.h>
static int g_fail, g_pass;
#define EXPECT(cond, ...)                                                                                    \
    do {                                                                                                     \
        if (cond) {                                                                                          \
            g_pass++;                                                                                        \
        } else {                                                                                             \
            g_fail++;                                                                                        \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);                                                    \
            printf(__VA_ARGS__);                                                                             \
            printf("\n");                                                                                    \
        }                                                                                                    \
    } while (0)
static float fabs_(float v)
{
    return v < 0 ? -v : v;
}
#define NEAR(a, b, tol) (fabs_((a) - (b)) <= (tol))
static void test_math(void)
{
    printf("[unit] TTC / stopping distance (spec examples)\n");
    float ttc;
    EXPECT(aeb_ttc(12.0f, 10.0f - 4.0f, 0.05f, &ttc) && NEAR(ttc, 2.0f, 1e-4f), "TTC 12/6 = %.4f", ttc);
    EXPECT(!aeb_ttc(12.0f, 0.0f, 0.05f, &ttc), "TTC with v_close = 0 must be invalid");
    EXPECT(!aeb_ttc(12.0f, -3.0f, 0.05f, &ttc), "TTC with receding target must be invalid");
    EXPECT(NEAR(aeb_stopping_distance(10.0f, 0.0f, 5.0f), 10.0f, 1e-3f), "d_brake 100/10");
    EXPECT(NEAR(aeb_stopping_distance(10.0f, 0.08f, 1e9f), 0.8f, 1e-3f), "d_delay 0.8 m");
    float ds = aeb_safe_distance(10.0f, 0.08f, 4.5f, 1.3f);
    EXPECT(NEAR(ds, 15.484f, 0.01f), "d_safe example = %.3f (expect 15.48)", ds);
    EXPECT(aeb_stopping_distance(0.0f, 0.08f, 2.0f) == 0.0f, "d_stop at v=0");
    EXPECT(aeb_stopping_distance(1.0f, 0.08f, 0.0f) > 1e6f, "a=0 must not divide by zero");
    float worst = 0;
    for (float x = 0.0001f; x < 1000.0f; x *= 1.37f) {
        float r = aeb_sqrtf(x), e = fabs_(r * r - x) / x;
        if (e > worst) worst = e;
    }
    EXPECT(worst < 1e-5f, "sqrt rel err %.2e", worst);
}
static void test_rd03d(void)
{
    printf("[unit] RD-03D protocol parser\n");
    EXPECT(rd03d_decode_value(0x0E, 0x03) == -782, "x 0x030E -> -782");
    EXPECT(rd03d_decode_value(0xB1, 0x86) == 1713, "y 0x86B1 -> +1713");
    EXPECT(rd03d_decode_value(0x10, 0x00) == -16, "speed 0x0010 -> -16");
    for (int v = -30000; v <= 30000; v += 777) {
        uint8_t lo, hi;
        rd03d_encode_value((int16_t)v, &lo, &hi);
        EXPECT(rd03d_decode_value(lo, hi) == v, "roundtrip %d", v);
    }
    const uint8_t frame[30] = {0xAA, 0xFF, 0x03, 0x00, 0x0E, 0x03, 0xB1, 0x86, 0x10, 0x00,
                               0x40, 0x01, 0,    0,    0,    0,    0,    0,    0,    0,
                               0,    0,    0,    0,    0,    0,    0,    0,    0x55, 0xCC};
    rd03d_parser_t ps;
    rd03d_frame_t f;
    int got = 0;
    rd03d_parser_init(&ps);
    const uint8_t junk[] = {0x12, 0xAA, 0xFF, 0x77, 0xAA};
    for (size_t i = 0; i < sizeof junk; i++) got += rd03d_parser_feed(&ps, junk[i], &f);
    for (int i = 0; i < 30; i++) got += rd03d_parser_feed(&ps, frame[i], &f);
    EXPECT(got == 1, "one frame after junk (got %d)", got);
    EXPECT(f.slot[0].present && f.slot[0].x_mm == -782 && f.slot[0].y_mm == 1713 &&
               f.slot[0].speed_cms == -16 && f.slot[0].res_mm == 320,
           "slot0 decode");
    EXPECT(!f.slot[1].present && !f.slot[2].present, "empty slots");
    uint8_t bad[30];
    memcpy(bad, frame, 30);
    bad[29] = 0x00;
    got = 0;
    for (int i = 0; i < 30; i++) got += rd03d_parser_feed(&ps, bad[i], &f);
    for (int i = 0; i < 30; i++) got += rd03d_parser_feed(&ps, frame[i], &f);
    EXPECT(got == 1 && ps.bad_footer == 1, "malformed frame rejected, next accepted (got %d bad %u)", got,
           (unsigned)ps.bad_footer);
    uint8_t cmd[12];
    const uint8_t want[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};
    EXPECT(rd03d_build_cmd(RD03D_CMD_MULTI, cmd, sizeof cmd) == 12 && memcmp(cmd, want, 12) == 0,
           "multi-target command bytes");
}
static void test_selection(void)
{
    printf("[unit] validation, tracking, target selection\n");
    aeb_params_t p;
    params_set_defaults(&p);
    params_apply_test_profile(&p);
    rd03d_frame_t f;
    memset(&f, 0, sizeof f);
    f.slot[0] = (rd03d_slot_t){true, false, 700, 1000, -100, 360};
    f.slot[1] = (rd03d_slot_t){true, false, 20, 2500, -100, 360};
    f.slot[2] = (rd03d_slot_t){true, false, 0, 80, -100, 360};
    radar_det_t d[3];
    uint32_t rej = 0;
    int n = radar_frame_to_detections(&f, &p, d, &rej);
    EXPECT(n == 2 && rej == 1, "2 valid detections, 1 rejected (n=%d rej=%u)", n, (unsigned)rej);
    tracker_t tk;
    tracker_init(&tk);
    for (int k = 0; k < 3; k++) {
        f.slot[0].y_mm = (int16_t)(1000 - k * 5);
        f.slot[1].y_mm = (int16_t)(2500 - k * 5);
        n = radar_frame_to_detections(&f, &p, d, &rej);
        tracker_update(&tk, d, n, (uint64_t)(k + 1) * 50000000ull, &p);
    }
    radar_track_t tr[RADAR_MAX_TRACKS];
    int nt = tracker_export(&tk, tr, RADAR_MAX_TRACKS);
    threat_t th;
    radar_select_threat(tr, nt, &p, &th);
    EXPECT(nt == 2, "two tracks (%d)", nt);
    EXPECT(th.valid && fabs_(th.x_m) < 0.1f && NEAR(th.range_m, 2.49f, 0.05f),
           "threat = in-path target (x=%.2f r=%.2f)", th.x_m, th.range_m);
    EXPECT(th.n_detected == 2 && th.n_in_corridor == 1, "detected 2, in corridor 1");
    EXPECT(NEAR(th.v_close_mps, 1.0f, 0.01f), "v_close from Doppler (%.2f)", th.v_close_mps);
    memset(&f, 0, sizeof f);
    f.slot[0] = (rd03d_slot_t){true, false, 600, 500, -200, 360};
    tracker_init(&tk);
    for (int k = 0; k < 3; k++) {
        n = radar_frame_to_detections(&f, &p, d, &rej);
        tracker_update(&tk, d, n, (uint64_t)(k + 1) * 50000000ull, &p);
    }
    nt = tracker_export(&tk, tr, RADAR_MAX_TRACKS);
    radar_select_threat(tr, nt, &p, &th);
    EXPECT(nt == 1 && !th.valid, "target beside the path is not a threat");
}
static void test_classify_and_sm(void)
{
    printf("[unit] risk classification, hysteresis, state machine\n");
    aeb_params_t p;
    params_set_defaults(&p);
    params_apply_test_profile(&p);
    threat_t th = {true, 1, 5.0f, 0.0f, 1.0f, true, 1, 1};
    risk_eval_t ev;
    EXPECT(aeb_classify(&th, &p, 1.0f, &ev) == RISK_NONE, "TTC 5 s -> NONE");
    th.range_m = 2.0f;
    EXPECT(aeb_classify(&th, &p, 1.0f, &ev) == RISK_WARNING, "TTC 2.0 -> WARNING");
    th.range_m = 1.5f;
    EXPECT(aeb_classify(&th, &p, 1.0f, &ev) == RISK_PARTIAL, "TTC 1.5 -> PARTIAL");
    th.range_m = 0.9f;
    EXPECT(aeb_classify(&th, &p, 1.0f, &ev) == RISK_STRONG, "TTC 0.9 -> STRONG");
    th.range_m = 0.5f;
    EXPECT(aeb_classify(&th, &p, 1.0f, &ev) == RISK_FULL, "TTC 0.5 -> FULL");
    th.range_m = 1.7f;
    EXPECT(aeb_classify(&th, &p, 1.0f, &ev) == RISK_WARNING &&
               aeb_classify(&th, &p, p.hyst_factor, &ev) == RISK_PARTIAL,
           "hysteresis band");
    th.v_close_mps = -0.5f;
    th.range_m = 0.8f;
    EXPECT(aeb_classify(&th, &p, 1.0f, &ev) == RISK_NONE && !ev.ttc_valid, "receding -> NONE");
    th.range_m = 0.2f;
    EXPECT(aeb_classify(&th, &p, 1.0f, &ev) == RISK_FULL, "proximity backstop");
    p.decel_measured[0] = 0.5f;
    p.decel_measured[1] = 0.8f;
    p.decel_measured[2] = 1.0f;
    th.v_close_mps = 2.0f;
    th.range_m = 5.0f;
    EXPECT(aeb_classify(&th, &p, 1.0f, &ev) == RISK_STRONG && ev.level_dist == RISK_STRONG &&
               ev.level_ttc == RISK_WARNING,
           "stopping-distance rule overrides large TTC (dist %s ttc %s)", risk_name(ev.level_dist),
           risk_name(ev.level_ttc));
    th.range_m = 3.0f;
    EXPECT(aeb_classify(&th, &p, 1.0f, &ev) == RISK_FULL && ev.level_dist == RISK_FULL,
           "only full braking can stop -> FULL");
    params_set_defaults(&p);
    params_apply_test_profile(&p);
    aeb_sm_t sm;
    aeb_sm_init(&sm, 0);
    aeb_sm_input_t in;
    memset(&in, 0, sizeof in);
    in.radar_ok = in.imu_ok = in.motor_ok = in.system_ok = true;
    uint64_t t = 0;
    const uint64_t ms = 1000000ull;
    EXPECT(sm.state == AEB_STOPPED, "boot state STOPPED");
    EXPECT(aeb_motor_cmd_for_state(sm.state, sm.has_driven, false, &p).type == MOTOR_DISABLE,
           "boot -> motors DISABLED");
    in.now_ns = t;
    in.cmd_go = true;
    aeb_sm_step(&sm, &in, &p);
    in.cmd_go = false;
    EXPECT(sm.state == AEB_NORMAL, "go -> NORMAL");
    in.level_enter = RISK_STRONG;
    in.level_exit = RISK_STRONG;
    t += 20 * ms;
    in.now_ns = t;
    aeb_sm_step(&sm, &in, &p);
    EXPECT(sm.state == AEB_STRONG_BRAKE, "escalation is immediate");
    in.level_enter = RISK_NONE;
    in.level_exit = RISK_STRONG;
    t += 20 * ms;
    in.now_ns = t;
    aeb_sm_step(&sm, &in, &p);
    t += 400 * ms;
    in.now_ns = t;
    aeb_sm_step(&sm, &in, &p);
    EXPECT(sm.state == AEB_STRONG_BRAKE, "no downgrade while exit-risk still >= state");
    in.level_exit = RISK_NONE;
    t += 20 * ms;
    in.now_ns = t;
    aeb_sm_step(&sm, &in, &p);
    t += 100 * ms;
    in.now_ns = t;
    aeb_sm_step(&sm, &in, &p);
    EXPECT(sm.state == AEB_STRONG_BRAKE, "downgrade waits DOWNGRADE_HOLD_MS");
    t += 250 * ms;
    in.now_ns = t;
    aeb_sm_step(&sm, &in, &p);
    EXPECT(sm.state == AEB_PARTIAL_BRAKE, "one step down after hold");
    in.level_enter = RISK_FULL;
    t += 20 * ms;
    in.now_ns = t;
    aeb_sm_step(&sm, &in, &p);
    EXPECT(sm.state == AEB_EMERGENCY_BRAKE, "FULL -> EMERGENCY");
    in.level_enter = RISK_NONE;
    in.level_exit = RISK_NONE;
    in.vehicle_stopped = true;
    t += 100 * ms;
    in.now_ns = t;
    aeb_sm_step(&sm, &in, &p);
    EXPECT(sm.state == AEB_EMERGENCY_BRAKE, "emergency held for minimum time");
    t += 500 * ms;
    in.now_ns = t;
    aeb_sm_step(&sm, &in, &p);
    EXPECT(sm.state == AEB_STOPPED, "EMERGENCY -> STOPPED (latched)");
    EXPECT(aeb_motor_cmd_for_state(sm.state, sm.has_driven, false, &p).type == MOTOR_BRAKE,
           "STOPPED after driving -> BRAKE held");
    t += 5000 * ms;
    in.now_ns = t;
    aeb_sm_step(&sm, &in, &p);
    EXPECT(sm.state == AEB_STOPPED, "no automatic resume");
    in.cmd_go = true;
    in.radar_ok = false;
    t += 20 * ms;
    in.now_ns = t;
    aeb_sm_step(&sm, &in, &p);
    EXPECT(sm.state == AEB_STOPPED, "resume refused with radar not OK");
    in.radar_ok = true;
    aeb_sm_step(&sm, &in, &p);
    in.cmd_go = false;
    EXPECT(sm.state == AEB_NORMAL, "resume with healthy sensors");
    in.radar_ok = false;
    t += 20 * ms;
    in.now_ns = t;
    aeb_sm_step(&sm, &in, &p);
    EXPECT(sm.state == AEB_RADAR_FAULT, "radar lost while driving -> RADAR_FAULT");
    in.radar_ok = true;
    in.motor_ok = false;
    aeb_sm_step(&sm, &in, &p);
    EXPECT(sm.state == AEB_MOTOR_FAULT &&
               aeb_motor_cmd_for_state(sm.state, true, false, &p).type == MOTOR_DISABLE,
           "motor fault -> drivers disabled");
    in.motor_ok = true;
    in.cmd_go = true;
    aeb_sm_step(&sm, &in, &p);
    EXPECT(sm.state == AEB_MOTOR_FAULT, "motor fault is latched");
    brake_verify_t bv;
    brake_verify_start(&bv, LVL_IDX_FULL, 0, 1.0f, true, &p);
    brake_verify_update(&bv, 30 * ms, -5.0f, true, &p);
    EXPECT(bv.status == VERIFY_RUNNING, "decel before VERIFY_DELAY ignored");
    brake_verify_update(&bv, 100 * ms, -1.5f, true, &p);
    EXPECT(bv.status == VERIFY_PASS, "verification PASS");
    brake_verify_start(&bv, LVL_IDX_FULL, 0, 1.0f, true, &p);
    for (int k = 1; k <= 60; k++) brake_verify_update(&bv, (uint64_t)k * 10 * ms, -0.1f, true, &p);
    EXPECT(bv.status == VERIFY_FAIL, "verification FAIL (no deceleration)");
    brake_verify_start(&bv, LVL_IDX_FULL, 0, 0.1f, true, &p);
    EXPECT(bv.status == VERIFY_SKIPPED, "not moving -> cannot verify (SKIPPED)");
}
typedef struct {
    aeb_state_t max_active, final_state;
    uint32_t faults_seen;
    bool collided;
    float min_gap;
    uint32_t rejects;
    float max_threat_abs_x;
    float worst_latency_ms;
} sc_result_t;
static int run_scenario(const sim_scenario_t *sc, bool verbose, sc_result_t *r)
{
    aeb_params_t p;
    params_set_defaults(&p);
    params_apply_test_profile(&p);
    sim_world_t w;
    sim_world_init(&w, sc);
    rd03d_parser_t ps;
    rd03d_parser_init(&ps);
    tracker_t tk;
    tracker_init(&tk);
    imu_proc_t ip;
    imu_proc_init(&ip);
    imu_proc_cal_set_zero(&ip);
    radar_state_t rs;
    memset(&rs, 0, sizeof rs);
    rs.link_ok = true;
    imu_state_t im;
    memset(&im, 0, sizeof im);
    vehicle_state_t vs;
    memset(&vs, 0, sizeof vs);
    aeb_sm_t sm;
    aeb_sm_init(&sm, 0);
    brake_sup_t bs;
    brake_sup_init(&bs);
    stop_det_t sd;
    memset(&sd, 0, sizeof sd);
    motor_cmd_t cmd = {MOTOR_DRIVE, p.cruise_duty};
    uint32_t latched = 0;
    unsigned frame_no = 0;
    memset(r, 0, sizeof *r);
    r->min_gap = 1e9f;
    aeb_state_t last_print = AEB_STATE_COUNT;
    const int steps = (int)(sc->duration_s * 1000.0f);
    for (int k = 1; k <= steps; k++) {
        const uint64_t now = (uint64_t)k * 1000000ull;
        sim_world_step(&w, sc, cmd, 0.001f);
        if (k % SIM_RADAR_PERIOD_MS == 0 && !sim_radar_silent(&w, sc)) {
            rd03d_frame_t f;
            uint8_t bytes[RD03D_FRAME_LEN];
            sim_make_radar_frame(&w, sc, &f, frame_no);
            rd03d_encode_frame(&f, bytes);
            if (sim_inject_corrupt_frame(&w, sc, frame_no)) bytes[29] ^= 0xFF;
            frame_no++;
            for (int i = 0; i < RD03D_FRAME_LEN; i++) {
                rd03d_frame_t df;
                if (rd03d_parser_feed(&ps, bytes[i], &df)) {
                    radar_det_t d[RD03D_NUM_SLOTS];
                    int n = radar_frame_to_detections(&df, &p, d, &rs.targets_rejected);
                    tracker_update(&tk, d, n, now, &p);
                    rs.n_tracks = (uint8_t)tracker_export(&tk, rs.tracks, RADAR_MAX_TRACKS);
                    rs.t_ns = now;
                    rs.frames_ok++;
                }
            }
            rs.frames_bad = ps.bad_footer;
        }
        if (k % 10 == 0 && !sim_imu_silent(&w, sc)) {
            float v6[6] = {w.a_ego_mps2, 0, 0, 0, 0, 0};
            imu_proc_sample(&ip, v6, now, &p, &im);
        }
        if (k % 20 == 0) {
            vs.stopped = stop_detect_update(&sd, now, false, 0.0f, imu_is_fresh(&im, now, &p), im.a_long,
                                            cmd.type, &p);
            if (sim_hang_now(&w, sc) && w.t_s >= sc->t_fault_s + (float)p.watchdog_timeout_ms / 1000.0f)
                latched |= FAULT_WDOG_THREAD;
            aeb_cycle_in_t in = {now,  &rs,     &im,
                                 &vs,  latched, sm.state == AEB_STOPPED && w.t_s < 0.6f && !sm.has_driven,
                                 false};
            aeb_cycle_out_t out;
            aeb_cycle(&sm, &p, &in, &out);
            cmd = out.cmd;
            if (sm.has_driven) r->faults_seen |= out.status.faults;
            if (out.status.threat.valid && fabs_(out.status.threat.x_m) > r->max_threat_abs_x)
                r->max_threat_abs_x = fabs_(out.status.threat.x_m);
            float v_ref = out.status.threat.valid ? out.status.threat.v_close_mps : 0.0f;
            brake_sup_on_state(&bs, out.status.state, now, v_ref, out.status.imu_ok, &p);
            int vst = brake_sup_tick(&bs, now, im.a_long, out.status.imu_ok, &p);
            if (vst == VERIFY_FAIL) latched |= FAULT_BRAKE_NO_DECEL;
            aeb_state_t st = out.status.state;
            if (st <= AEB_EMERGENCY_BRAKE && st > r->max_active) r->max_active = st;
            if (verbose && st != last_print) {
                printf("    t=%5.2fs %-15s gap=%6.2fm v=%4.2f ttc=%5.2f risk=%s\n", w.t_s, aeb_state_name(st),
                       out.status.threat.valid ? out.status.threat.range_m : -1.0f, w.v_ego_mps,
                       out.status.ttc_valid ? out.status.ttc_s : -1.0f, risk_name(out.status.risk));
                last_print = st;
            }
            r->final_state = st;
        }
    }
    r->collided = w.collided;
    r->min_gap = w.min_gap_m;
    r->rejects = rs.targets_rejected + ps.bad_footer;
    bool ok = true;
    if (r->max_active < sc->max_active_min || r->max_active > sc->max_active_max) ok = false;
    if (sc->final_state >= 0 && r->final_state != (aeb_state_t)sc->final_state) ok = false;
    if (sc->require_no_collision && r->collided) ok = false;
    if ((r->faults_seen & sc->require_faults) != sc->require_faults) ok = false;
    if (sc->require_rejects && r->rejects == 0) ok = false;
    if (r->max_threat_abs_x > p.corridor_half_width_m + 1e-3f) ok = false;
    return ok ? 0 : 1;
}
int selftest_scenarios(bool verbose)
{
    int fails = 0;
    printf("[scenarios] closed-loop AEB tests (sim model, 1 ms step)\n");
    for (int i = 1; i <= sim_num_scenarios(); i++) {
        const sim_scenario_t *sc = sim_get_scenario(i);
        sc_result_t r;
        if (verbose) printf("  -- TEST %d: %s\n", sc->id, sc->name);
        int f = run_scenario(sc, verbose, &r);
        printf("  TEST %2d %-48s %s  (max=%s final=%s gap=%s%.2fm faults=0x%03x rej=%u)\n", sc->id, sc->name,
               f ? "FAIL" : "PASS", aeb_state_name(r.max_active), aeb_state_name(r.final_state),
               r.collided ? "COLLISION " : "", r.min_gap > 1e8f ? 0.0f : r.min_gap, (unsigned)r.faults_seen,
               (unsigned)r.rejects);
        if (f) printf("           expected: %s\n", sc->expect_text);
        fails += f;
        if (f)
            g_fail++;
        else
            g_pass++;
    }
    return fails;
}
int selftest_run_all(bool verbose)
{
    g_fail = g_pass = 0;
    test_math();
    test_rd03d();
    test_selection();
    test_classify_and_sm();
    selftest_scenarios(verbose);
    printf("\nSELFTEST: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail;
}
