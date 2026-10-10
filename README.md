# QNX Autonomous Emergency Braking (AEB) Controller — Raspberry Pi 5

QNX eHACK 2026, problem statement #2. RD-03D radar (UART), MPU6050 IMU (I²C), 2 × BTS7960 (driver #1 = FRONT axle M1+M3, driver #2 = REAR axle M2+M4), master switch.
**Wiring: see `WIRING.md`. Design (spec steps 1–9): see `docs/ARCHITECTURE.md`.**

## Quick start (final system: AEB + camera + dashboard)
On the PC (Ubuntu, QNX SDP 8.0 sourced, OpenCV built once per `docs/VISION.md`):
```sh
make                                              # aeb, aeb_guard, aeb_dash
make vision OPENCV_ROOT=~/qnx_workspace/staging   # aeb_vision (camera + OpenCV)
make deploy-all OPENCV_ROOT=~/qnx_workspace/staging TARGET=qnxuser@<pi-ip>
```
On the Pi (as root, `cd /data/home/qnxuser/aeb`):
```sh
./master.sh                    # EVERYTHING: dashboard + camera + guard + AEB; Ctrl-C = safe stop
./master.sh bg                 # same, untethered (unplug Ethernet); ./master.sh stop / ./master.sh status
nohup ./autostart.sh 15 > /dev/null 2>&1 &   # same, untethered (unplug Ethernet); ./stop_aeb.sh to stop
```
Open **http://<pi-ip>:8080** in a browser (laptop or phone, same network). Camera lines
(`OBJECT DETECTED: PERSON …`) appear in `vision.log`, in the AEB console/log and on the dashboard.
Details: `docs/DASHBOARD.md`, `docs/VISION.md`.

## Default behaviour (full mode)
1. Start: `./start_uart.sh` then `./aeb -m full -c aeb.conf` (add `-p soft` if the motors do not turn with hardware PWM).
2. About 2 s after the radar is healthy and the path is clear, **the car starts driving by itself** (60 % duty).
3. Approaching object in the path: WARNING, then PARTIAL braking (based on TTC), then **immediate full stop when it is closer than 1.0 m** (`proximity_stop_m`).
4. The car stays STOPPED with the motors braked. Type `go` to drive again, or `quit`.
5. The IMU is optional (`imu_fault_policy = 1`): without it the car still runs; only brake verification is skipped.

Tune in `aeb.conf`: `proximity_stop_m` (stop distance), `cruise_duty` (speed), `auto_start` (0 = wait for `go`).

## Files
```
src/config.h            all parameters + GPIO map (defaults)      src/params.c   runtime overrides (-c file)
src/aeb_logic.c         TTC, stopping distance, risk, state machine, brake verification (pure C)
src/aeb_cycle.c         one 50 Hz control cycle (pure, used by the thread AND the tests)
src/rd03d_proto.c       RD-03D frame parser            src/radar_track.c  validation, tracking, threat selection
src/radar_rd03d.c       UART thread (/dev/ser11)       src/imu_mpu6050.c  I2C thread (/dev/i2c1), src/imu_proc.c filter/impact
src/aeb_controller.c    prediction thread (prio 58)    src/brake.c        brake thread (prio 58, 63 on emergency pulse)
src/motor_bts7960.c     BTS7960 via QNX rpi_gpio       src/gpio_rpi.c     /dev/gpio/msg message client
src/watchdog.c          watchdog (prio 62), deadline timer, guard link
src/vehicle_state.c     ego-speed source, stopped detection   src/camera_if.c reads camera results (shared memory)
src/logger.c            lock-free ring -> CSV + console + slogger2
src/sim_model.c sim_rt.c selftest.c   12 test scenarios + real-time simulation
src/test_modes.c        motor test + braking calibration   src/main.c  modes, CLI, shutdown
guard/aeb_guard.c       separate safety process: stops motors if aeb crashes/hangs
vision/aeb_vision.cpp   IMX708 camera (libcamapi) + OpenCV MobileNet-SSD, low priority -> /aeb_vision shm
vision/models/          MobileNet-SSD model (Caffe)        docs/VISION.md  camera + OpenCV steps
src/telemetry.c         live state -> shm /aeb_telemetry, remote commands <- /aeb_control
dash/aeb_dash.c         dashboard web server (port 8080)   dash/www/index.html  dashboard UI
scripts/                start_all.sh, autostart.sh, stop_aeb.sh, start_uart.sh, run_vision.sh, run_dash.sh,
                        camera_setup.sh, check_camera.sh, run_aeb.sh, run_all_sim.sh, check_target.sh
tools/plot_log.py       host-side timeline plot of the CSV log
```

## Build (host PC with QNX SDP 8.0)
```sh
source ~/qnx800/qnxsdp-env.sh          # Windows: qnxsdp-env.bat
make                                   # -> build/aeb  build/aeb_guard   (qcc -Vgcc_ntoaarch64le, links -lslog2)
make host-test                         # optional: logic tests on the PC with gcc
make vision OPENCV_ROOT=...            # optional camera detector -> build/aeb_vision (see docs/VISION.md)
```

## Deploy
```sh
make deploy TARGET=qnxuser@<pi-ip>     # or: scp build/aeb build/aeb_guard scripts/*.sh qnxuser@<pi-ip>:/data/home/qnxuser/aeb/
ssh qnxuser@<pi-ip>;  su  (root);  cd /data/home/qnxuser/aeb;  chmod +x aeb aeb_guard *.sh
```

## Run — in this order
| Step | Command | What you should see |
|---|---|---|
| 0 | `./check_target.sh` | /dev/gpio/msg, /dev/i2c1, /dev/ser11 present |
| 1 | `./aeb -m selftest` | `SELFTEST: 144 passed, 0 failed` (no hardware needed) |
| 2 | `./run_all_sim.sh` | 12 × PASS (real-time threads, simulated sensors, motors untouched) |
| 3 | `./start_uart.sh` then `./aeb -m radar` | frames ~every 50–100 ms; walk in front: x/y/v_close; note `period=` |
| 4 | `./aeb -m imu` | keep still 2 s → `CAL`; push car forward → `a_fwd` positive (else set `imu_forward_axis/sign`) |
| 5 | `./aeb -m sensor` then type `go` | full AEB logic, motors NOT driven; walk toward radar → WARNING…EMERGENCY |
| 6 | `./aeb -m motor -y` (wheels off ground, master ON) | FRONT axle fwd, REAR axle fwd, all fwd + brake; if one wheel of an axle turns backward swap that motor's wires (WIRING.md §8) |
| 7 | `./aeb -m calib -y` (open floor) | measured decel per level → `aeb_calib.conf` |
| 8 | `./aeb_guard &` then `./aeb -m full -G -c aeb_calib.conf` → type `go` | car drives; brakes before the obstacle; IMU confirms decel |

| 9 (optional) | `./run_vision.sh -q &` before step 8 | AEB log: `camera … connected`, `camera: person ahead`, `EMERGENCY_BRAKE: camera CONFIRMS person` (docs/VISION.md) |

CLI (full/sensor): `go`/`resume`, `stop`, `estop`, `clear` (impact latch), `status`, `params`, `quit`. Ctrl-C = safe stop.
Options: `-c conf` `-l log.csv` `-e none|radar_static|imu|sim` (ego-speed source) `-p hw|soft|none` (PWM backend) `-u /dev/serN` `-i /dev/i2cN` `-q` `-P`.

## Debug on the Pi
`pidin -p aeb threads` (thread names/priorities) · `slog2info -b aeb` (events) · `tail -f aeb_log.csv` · `status` in the CLI.
Plot a run on your PC: `python3 tools/plot_log.py aeb_log.csv` → `aeb_timeline.png` (see `docs/example_sim_timeline.png`).

| Symptom | Fix |
|---|---|
| `cannot open /dev/gpio/msg` | start the `rpi_gpio` resource manager (QSTI starts it by default) |
| `hardware PWM setup rejected` | automatic fallback to software PWM 100 Hz - OK |
| radar: no frames | TX/RX crossed? 5 V on radar? `./start_uart.sh`; `stty < /dev/ser11` shows 256000 |
| `no answer at 0x68` | SDA/SCL swapped, AD0 not GND, VCC 3.3 V missing |
| `go` refused | radar not fresh, IMU still calibrating, or in-path object closer than PARTIAL range |
| MOTOR_FAULT after braking | brake verification failed: master switch off? battery low? `verify_min_decel_*` too high? |

## Known limitations (be honest in the demo)
1. RD-03D is Doppler-based: **stationary objects are invisible while the car is stopped**; seen only while approaching. Hence no auto-resume.
2. No wheel encoder: ego speed is not measured (decisions use radar closing speed, exact for static obstacles).
3. Model decelerations (2.0/4.5/7.5 m/s²) are not physical until `-m calib` measures them; TTC 0.6 s is a prototype value.
4. Motor stall/rated current unknown → measure (WIRING.md §9) before sizing fuse/switch.
5. Camera (IMX708 + MobileNet-SSD, docs/VISION.md) only confirms/classifies; radar is the only ranging sensor and braking trigger.
6. RD-03D frame rate / behaviour with no target must be checked with `-m radar`; tune `radar_timeout_ms`, `t_delay_s`.
