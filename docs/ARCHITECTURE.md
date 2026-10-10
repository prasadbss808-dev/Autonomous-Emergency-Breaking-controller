# QNX AEB — architecture and design rationale

This document covers steps 1–9 of the project specification. The code is in `src/` and `guard/`. Wiring is in `WIRING.md`, and build/run/test instructions are in `README.md`.

Problem statement #2 (QNX eHACK 2026): *"Detect imminent collision and trigger braking within a deterministic response deadline… Emergency braking must preempt non-critical workloads and execute within a specified deadline."*

---

## 1. Architecture review

The chain is **SENSE → ESTIMATE → PREDICT → DECIDE → BRAKE → VERIFY**, with one QNX thread per stage and a watchdog plus an external guard process around them:

```
 RD-03D ──UART──► [radar 50]──┐                         ┌──► [motor/pwm 57] ──GPIO──► BTS7960 #1 ─► M1 M3 (front)
                              ├─► shared snapshots ─► [prediction 58] ══MsgSend / pulse 63══► [brake 58] ─┤
 MPU6050 ──I2C──► [imu 50] ───┤        (mutex,              │                              │           └──► BTS7960 #2 ─► M2 M4 (rear)
                              │    priority inheritance)    │                     verify with IMU (closed loop)
           [vehicle 30] ──────┘                             ▼
                                                   [logger 10] ─► CSV + console + slogger2
 [watchdog 62] heartbeats · braking-deadline timer · direct GPIO safe path ──pulse──► aeb_guard (separate process)
```

Key decisions:

- **The decision core is pure C** (`aeb_logic.c`, `aeb_cycle.c`, `radar_track.c`, `rd03d_proto.c`, `imu_proc.c`). It has no OS calls, so the same code runs in the QNX threads, in the target self-test (`aeb -m selftest`) and in a host unit test. What is tested is what runs.
- **The emergency path has no locks:** pulse at priority 63, then `motor_apply()`, then GPIO messages to `rpi_gpio`.
- **There are three independent safe paths:** brake thread (normal), watchdog writing the GPIOs directly (thread failure), and the `aeb_guard` process (whole-process crash or hang). Hardware pull-downs and the master switch sit underneath.
- **Closed loop:** every brake command is checked against IMU deceleration. No deceleration means MOTOR_FAULT.
- **Change from the conceptual design:** the radar (about 10–20 Hz frames) is polled every 10 ms and the decision runs at 50 Hz (20 ms), as specified. The prediction-to-brake call for normal decisions uses a synchronous `MsgSend` with a reply. That reply provides a measured actuation timestamp for "response latency" logging.

## 2. Confirmed information

| Item | Value | Source |
|---|---|---|
| Platform | Raspberry Pi 5, 4 × Cortex-A76, QNX SDP 8.0 | user |
| RD-03D UART | 256000 baud 8N1, `/dev/ser11` | user + ESPHome rd03d docs |
| RD-03D report frame | `AA FF 03 00` + 3 × 8-byte targets + `55 CC` (30 B); x, y mm, speed cm/s, bit 15 = 1 means positive; speed + = receding; x + = right | ESPHome `rd03d.cpp` (verified implementation) |
| RD-03D commands | `FD FC FB FA` len cmd `04 03 02 01`; multi-target `0x0090` | ESPHome `rd03d.cpp` |
| RD-03D power / IO | 5 V supply (~92 mA typ, ≥ 200 mA peak), 3.3 V UART, ~8 m range, ±60° FOV, **cannot see non-moving objects** | ESPHome docs, module guides |
| MPU6050 | `/dev/i2c1`, address 0x68, 100 Hz; register map (0x3B…, 0x6B, 0x75 = 0x68) | user + MPU-6050 register map |
| GPIO control on QNX | `rpi_gpio` resource manager, `/dev/gpio/msg`, `_IO_MSG` with `RPI_GPIO_IOMGR`, subtypes SET_SELECT/WRITE/READ/PWM_SETUP/PWM_DUTY/PUD | QNX hardware-component-samples (`rpi_gpio.h`, motor/servo samples) |
| Hardware PWM pins | GPIO 12, 13, 18, 19; 12 and 13 on different channels | QNX GPIO docs |
| Pi 5 UART0 | GPIO14/15, `msix-rp1 -m set 25l`, `gpio-rp1 set 14,15 a4`, `devc-serpl011-rpi5 … 0x1f00030000,185` | QNX "Configuring UART" guide |
| QNX I²C API | `devctl(DCMD_I2C_SENDRECV / DCMD_I2C_SEND)` with `i2c_sendrecv_t` / `i2c_send_t` | QNX `rpi_i2c` sample |
| IBT-2 header | 1 RPWM, 2 LPWM, 3 R_EN, 4 L_EN, 5 R_IS, 6 L_IS, 7 VCC, 8 GND; control level 3.3–5 V | module datasheet |
| Motors | ~3–12 V, ~60–200 RPM; driver 1 = FRONT axle (M1 FL + M3 FR), driver 2 = REAR axle (M2 RL + M4 RR) | user |
| Prototype parameters | TTC_CRITICAL 0.6 s, decel 2.0/4.5/7.5 m/s², SF 1.3, delay 80 ms, WDOG 10/120 ms, deadline 50 ms, PWM 1000/100 Hz, cruise 60 % | user (all configurable) |

## 3. Missing — not guessed (must be measured or supplied)

| # | Missing item | How the code handles it |
|---|---|---|
| 1 | Motor rated/stall current | not used in code; WIRING §9 measurement procedure; `motor_max_duty` limit |
| 2 | Real braking deceleration per level | model values until `aeb -m calib -y` writes `decel_measured_*` |
| 3 | Ego speed (no wheel encoder) | `ego_speed` source is selectable (`none` default; `radar_static` explicit assumption; `imu` estimate; `encoder` hook). Decisions use radar **closing** speed, which is valid without ego speed (§7) |
| 4 | RD-03D actual frame rate on your unit | measured and printed by `aeb -m radar` → set `radar_timeout_ms`, `t_delay_s` |
| 5 | Whether the RD-03D sends frames when no target is present | `aeb -m radar` with an empty room shows it; if frames stop, raise `radar_timeout_ms` |
| 6 | End-to-end latency (sensor → wheel torque) | software part is logged per cycle (`cmd_latency_us`); mechanical part is in `T_DELAY_S` |
| 7 | BTS7960 brake behaviour on your motors | brake = short-brake (explicit); measured by calib; coast reference also measured |
| 8 | IMU mounting axis | `imu_forward_axis/sign`; verify with `aeb -m imu` |
| 9 | Camera ↔ AEB link | IMX708 + OpenCV run on the same Pi in the separate low-priority process `aeb_vision`; result passed through POSIX shared memory `/aeb_vision` (seqlock, wait-free read). Camera only confirms/classifies; loss = informational fault, never "no obstacle" (docs/VISION.md) |
| 10 | Whether Pi 5 `rpi_gpio` accepts hardware-PWM setup | automatic fallback to the software-PWM thread (100 Hz) |
| 11 | Vehicle width / corridor | `corridor_half_width_m` (default 0.20 m) |

## 4. Thread architecture (priorities, policies, CPUs, periods)

| Thread | Policy / prio | CPU (runmask) | Trigger | Critical (watchdog) | Job |
|---|---|---|---|---|---|
| watchdog | FIFO 62 | CPU3 (0x8) | 10 ms timer pulse + deadline one-shot timer | — | heartbeats, deadline, stale-data backstop, guard heartbeat |
| brake | FIFO 58 (**63** while handling the emergency pulse) | CPU0 (0x1) | messages / pulses + 10 ms verify tick | yes | apply motor command, emergency brake, IMU verification |
| prediction | FIFO 58 | CPU1 (0x2) | 20 ms timer pulse | yes | `aeb_cycle()`, state machine, command brake |
| motor_pwm | FIFO 57 | CPU0 (0x1) | 20 ms (read-back) or PWM period (soft PWM) | yes | software PWM fallback, output read-back, safe-state refresh |
| radar | FIFO 50 | CPU2 (0x4) | 10 ms timer pulse, non-blocking `readcond()` | yes | UART → parser → validation → tracking → snapshot |
| imu | FIFO 50 | CPU2 (0x4) | 10 ms timer pulse | yes | I²C read → calibration → filter → impact → snapshot |
| sim (sim mode only) | FIFO 50 | CPU2 | 2 ms | yes | simulated radar bytes / IMU samples, fault injection |
| vehicle | RR 30 | CPU2 | 20 ms | no (warn) | ego speed source, stopped detection, camera status |
| logger | RR 10 | CPU2 | 50 ms | no (warn) | drain the lock-free ring → CSV, console, slogger2 |
| main / CLI, signals | RR 10 | any | stdin / `sigwaitinfo` | — | operator commands, orderly shutdown |
| **aeb_guard** (process) | FIFO 61 | any | 20 ms timer + heartbeat pulses | — | process-death / hang protection |
| **aeb_vision** (process, optional) | RR 8 (+ OpenCV workers, inherited) | CPU2-3 (0xC, inherited) | camera viewfinder callback | — (heartbeat in shm) | IMX708 NV12 → MobileNet-SSD → shared memory; complementary only |
| **aeb_dash** (process, optional) | RR 9 | any | HTTP requests | — | dashboard web server; reads /aeb_telemetry + /aeb_vision, writes /aeb_control (GO/STOP/E-STOP) |

The priority order from the specification is kept (63 > 62 > 58 > 57 > 50 > 30 > 10). Prediction and brake are both 58, but they run on different CPUs. The emergency pulse runs the brake thread at 63, above everything except the kernel. Thread names show up in `pidin -p aeb threads`.

**IPC used:** timer pulses (`timer_create` + `SIGEV_PULSE_INIT`), `MsgSend` / `MsgReceive` / `MsgReply` (decision → brake, with a time-bounded `TimerTimeout`), `MsgSendPulse` at priority 63 (emergency, fault), `name_attach` / `name_open` + `_PULSE_CODE_DISCONNECT` (guard), resource-manager messages (`rpi_gpio`), `devctl` (I²C), mutexes with priority inheritance (snapshots), and C11 atomics (heartbeats, faults, ring buffer).

## 5. Data structures (`src/aeb_types.h`)

- `radar_track_t`: id (tracker), slot, x, y, range, Doppler radial speed (+ validity), range-rate closing speed (+ validity), **closing speed used**, resolution, hits/misses, confirmed, in_corridor, timestamp.
- `radar_state_t`: link_ok, **t_ns of the last valid frame**, frame counters (ok / bad / rejected), measured frame period, tracks[6].
- `threat_t`: valid, id, range, x, v_close, closing, n_detected, n_in_corridor. This keeps "detected" separate from "collision threat".
- `imu_state_t`: valid, t_ns, ax..gz (filtered, offset-free), a_long (forward), a_dyn_g (impact metric), impact, calibrated, counters.
- `vehicle_state_t`: ego_valid, ego_speed, **source**, stopped, t_ns.
- `aeb_status_t`: state, risk, threat, TTC (+ valid), d_safe[PARTIAL, STRONG, FULL], unavoidable, radar_ok, imu_ok, motor_ok, system_ok, degraded, faults, ego, a_long.
- `motor_cmd_t`: DISABLE / DRIVE(duty) / BRAKE.
- No field requires unavailable data: there is no confidence value (the RD-03D does not provide one) and ego speed is optional.

## 6. State machine (`aeb_sm_step`)

```
            go (sensors OK, path clear)
  STOPPED ─────────────────────────────► NORMAL ─► WARNING ─► PARTIAL_BRAKE ─► STRONG_BRAKE ─► EMERGENCY_BRAKE
  (boot: coast)   ◄── stop ──                ◄──────── one step down after DOWNGRADE_HOLD_MS ────────┘        │
     ▲                                          (exit thresholds = enter × HYST_FACTOR 1.2)                    │
     └──────────────── EMERGENCY held ≥ 500 ms, then vehicle stopped (or 3 s) ──────────────────────────────────┘
  Any active state ── radar stale ──► RADAR_FAULT   (brake; resume needs 'go' + healthy radar)
                   ── IMU lost (policy 0) ──► IMU_FAULT (brake; policy 1 = degraded drive ≤ 35 %)
                   ── impact ──► EMERGENCY_BRAKE ──► STOPPED (latched, 'clear' + 'go')
  Any state ── brake verification FAIL / GPIO error ──► MOTOR_FAULT (drivers DISABLED, restart required)
            ── watchdog / deadline miss ──► SYSTEM_FAULT (brake via the direct path, restart required)
```

- **Escalation is immediate.** Downgrade happens one level at a time, only after the risk has stayed below the **exit** threshold (enter × 1.2) for 300 ms. This prevents oscillation.
- **EMERGENCY is never downgraded.** It always ends in STOPPED, and STOPPED never auto-resumes by default. This matters with a Doppler radar: after stopping, the obstacle becomes invisible.
- Motor command per state: NORMAL / WARNING DRIVE 60 %, PARTIAL DRIVE 30 %, STRONG DRIVE 10 %, EMERGENCY / faults BRAKE, STOPPED BRAKE (or DISABLE at boot), MOTOR_FAULT DISABLE.

## 7. TTC and stopping-distance algorithm (`aeb_classify`)

Each 20 ms cycle:

1. Freshness: radar age ≤ `radar_timeout_ms` (250), IMU age ≤ `imu_timeout_ms` (100). Stale data is never used.
2. Validation (per radar slot): 0.15 ≤ r ≤ 25 m, y > 0, |x| ≤ 8 m, |v| ≤ 10 m/s, no sentinel speed. Rejected data is counted and can never brake.
3. Tracking: nearest-neighbour association (gate 0.6 m), confirmed after 2 hits, dropped after 3 misses, low-pass on range and speed.
4. Corridor: |x| ≤ `corridor_half_width_m`. Only confirmed in-corridor tracks are threat candidates. The candidate with the highest risk level is selected, then smallest TTC, then smallest range. It is **not** simply the nearest target.
5. Closing speed `v = −v_radial` (Doppler, RD-03D sign: − = approaching). If Doppler is invalid, `v = −dr/dt` (range rate).
6. `TTC = r / v` **only if** `v > V_CLOSE_MIN` (0.05 m/s). Otherwise there is no TTC (no division by zero or by a negative value).
7. For each level i: `d_safe_i = SF × ( v·T_delay + v² / (2·a_i) )`, with `a_i` = measured deceleration if calibrated, else the model value (2.0 / 4.5 / 7.5).
   - Using **closing** speed is exact for a constant-velocity target. Braking at a_i reduces the closing speed at a_i, so the gap consumed is `v·T + v²/2a`. For a static obstacle `v` = ego speed, so no wheel encoder is needed. A target that is itself braking is covered only by SF; this is a documented limitation.
8. Levels:

| Rule | → level |
|---|---|
| TTC ≤ 0.6 s, or r ≤ d_safe_STRONG (only FULL can stop), or in-path target closer than 0.30 m | **FULL** → EMERGENCY_BRAKE |
| TTC ≤ 1.0 s, or r ≤ d_safe_PARTIAL (partial no longer enough) | **STRONG** |
| TTC ≤ 1.6 s, or r ≤ 1.25 × d_safe_PARTIAL | **PARTIAL** |
| TTC ≤ 2.5 s, or r ≤ 1.6 × d_safe_PARTIAL | **WARNING** |
| otherwise | NONE → NORMAL |

   `unavoidable = r ≤ d_safe_FULL` (logged). TTC is *time to impact at current closing speed*. Stopping time and distance depend on speed, deceleration and delay, which is why both rules are used.
9. Worked example from the specification: v = 10 m/s, a = 4.5, T = 0.08, SF = 1.3 → d_delay 0.8 m, d_brake 11.11 m, d_stop 11.91 m, **d_safe 15.48 m** (verified in the self-test).

## 8. Motor-control architecture

- Driver #1 = front axle, driver #2 = rear axle (right-hand motor of each axle wired crossed, WIRING.md §8). Both drivers always receive the same command (all four wheels together). The axle split keeps braking left/right symmetric if one driver fails. Per driver: RPWM (hardware PWM GPIO12/13, 1 kHz mark-space), LPWM held LOW, EN (R_EN + L_EN tied).
- **Command level ≠ deceleration.** PARTIAL/STRONG lower the duty (drive-reduction braking: the PWM off-phase shorts the winding). FULL is explicit short-brake (EN = 1, both IN low). COAST is EN = 0. The real decelerations are **measured** (`-m calib`) and replace the model values. Reverse plugging is deliberately not used: it causes high current and the motor currents are unknown.
- Write order guarantees no shoot-through or unwanted drive. To stop: RPWM = 0, then LPWM = 0, then EN = 1. To drive: LPWM = 0, then EN = 1, then duty. If a write fails while braking, EN goes low.
- Backends: hardware PWM via `rpi_gpio` (default) → automatic fallback to the software-PWM thread at 100 Hz → dry-run (sim/sensor tests).
- Safe defaults: all pins LOW at start (drivers OFF) and DISABLE at exit (`atexit`, signal thread, crash handler). The safe-latch is permanent until restart.

## 9. Safety and fault architecture

| Failure | Detection | Reaction |
|---|---|---|
| Radar silent / UART error | freshness 250 ms; read error | RADAR_FAULT → brake; UART auto-reopen; resume only with `go` |
| Malformed radar data | footer check, plausibility, tracking confirmation | dropped and counted; never brakes |
| IMU failure / frozen | I²C errors ×5, 50 identical samples, freshness 100 ms, bad calibration | IMU_FAULT → brake (policy 0) or degraded ≤ 35 % (policy 1) |
| Impact | \|a\| ≥ 3 g for 3 consecutive samples | EMERGENCY → STOPPED latched |
| Brake has no effect | IMU decel < threshold within 500 ms (when v ≥ 0.3 m/s) | MOTOR_FAULT → drivers DISABLED |
| GPIO/driver signal fault | write error, read-back mismatch ×3 | MOTOR_FAULT |
| Critical thread hung | heartbeat older than 120 ms | SYSTEM_FAULT: latch + **direct GPIO safe write** + pulse to brake |
| Braking deadline | one-shot timer (50 ms) vs brake acknowledgement | FAULT_DEADLINE_MISS → SYSTEM_FAULT |
| Brake thread unreachable | `MsgSend` with `TimerTimeout` (50 ms) | direct safe write + deadline fault |
| Driving on stale radar | watchdog: DRIVE with radar age > 370 ms | SYSTEM_FAULT |
| Process crash / SIGKILL / hang | `aeb_guard`: `_PULSE_CODE_DISCONNECT` or heartbeat > 150 ms | guard brakes then disables; kills a hung process |
| Pi boot / wire off | — | 10 kΩ pull-downs keep the drivers OFF |
| Anything | — | master switch (hardware cut of motor power) |

Never "sensor failed → assume clear": an invalid sensor makes the state machine brake, and a fault is never treated as "no obstacle".
