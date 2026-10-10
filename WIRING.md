# WIRING — pin-to-pin connections

Raspberry Pi 5 (QNX) · RD-03D radar · MPU6050 IMU · 2 × BTS7960 (IBT-2) · 4 geared DC motors · master switch · 12 V motor battery

All GPIO numbers are **BCM numbers**; `[n]` is the **physical pin** on the Pi 5 40-pin header (pin 1 = the corner pin nearest the SD-card end, marked with a square pad; odd pins are on the inner row).

---

## 0. Read first: rules that protect the Pi

1. **Nothing above 3.3 V ever touches a Pi GPIO pin.** 12 V goes only to the drivers' **B+ / B−** and the motors.
2. **The Pi is never powered from the motor battery.** Use the official 27 W USB-C supply or a USB-C PD power bank (5 V, ≥ 3 A).
3. **Fuse and master switch go in the battery POSITIVE (+) line, never in the negative line.** If the negative line is switched off, motor current tries to return through the Pi ground wire and can destroy the Pi.
4. **Use one ground connection between the Pi and the motor side.** A single thin wire runs from the Pi to the star point (section 7). The IBT-2 header GND pins are **not** wired to the Pi.
5. **Do not connect R_IS / L_IS (IBT-2 pins 5 and 6) to anything.** These current-sense outputs can rise to about 5 V under fault conditions, which would damage a GPIO pin.
6. Wire everything with the battery **disconnected**. Power-up and power-down order is in section 10.

> **Your earlier "abnormal Pi power/LED" problem.** It is usually caused by one of these: IBT-2 VCC tied to the Pi's 5 V rail while the 12 V side was powered or unpowered (back-powering), R_IS/L_IS wired to GPIOs, two or more ground paths so motor current flowed through the Pi, or the Pi sharing the motor battery (brown-out on motor start). The wiring below removes all four.

---

## 1. Parts list

| Qty | Part | Notes |
|---|---|---|
| 1 | Raspberry Pi 5 with QNX 8.0 image | own USB-C supply / power bank |
| 1 | RD-03D 24 GHz radar (Ai-Thinker) | 5 V supply, **3.3 V UART** |
| 1 | MPU6050 module (GY-521) | powered from 3.3 V |
| 2 | BTS7960 / IBT-2 driver module | #1 = FRONT axle, #2 = REAR axle |
| 4 | Geared DC motor | M1 FL, M2 RL, M3 FR, M4 RR |
| 1 | 12 V battery | 3S Li-ion/LiPo or 12 V SLA |
| 1 | Master switch | **DC** rating ≥ fuse rating, in battery + line |
| 1 | Inline blade-fuse holder + fuse | sized in section 9 |
| 2 | Screw-terminal / lever (Wago) blocks | "+12 V SWITCHED" bus and "STAR GND" point |
| 4 | 10 kΩ resistors (¼ W) | **strongly recommended** pull-downs, section 6 |
| 4 | 100 nF ceramic capacitors | recommended, across each motor's terminals |
| 2 | 470–1000 µF / 25 V electrolytic | recommended, B+/B− at each driver (mind the polarity) |
| — | Wire: 1.0–1.5 mm² (16–18 AWG) for battery/motors, Dupont jumpers for logic | |

---

## 2. Raspberry Pi 5 header — every pin used

| Pi pin | Function | Connects to |
|---|---|---|
| **[1]** | 3.3 V | MPU6050 **VCC** |
| **[2]** | 5 V | RD-03D **5V** |
| **[3]** | GPIO2 / I2C1 SDA | MPU6050 **SDA** |
| **[5]** | GPIO3 / I2C1 SCL | MPU6050 **SCL** |
| **[6]** | GND | RD-03D **GND** |
| **[8]** | GPIO14 / UART0 TXD | RD-03D **RX** |
| **[9]** | GND | MPU6050 **GND** (and MPU6050 **AD0**) |
| **[10]** | GPIO15 / UART0 RXD | RD-03D **TX** |
| **[13]** | GPIO27 | IBT-2 #2 (REAR) pin 2 **LPWM** |
| **[15]** | GPIO22 | IBT-2 #1 (FRONT) pin 2 **LPWM** |
| **[16]** | GPIO23 | IBT-2 #1 (FRONT) pin 3 **R_EN** (jumpered to pin 4 L_EN) |
| **[17]** | 3.3 V | IBT-2 #1 pin 7 **VCC** → daisy-chained to IBT-2 #2 pin 7 **VCC** |
| **[18]** | GPIO24 | IBT-2 #2 (REAR) pin 3 **R_EN** (jumpered to pin 4 L_EN) |
| **[30]** | GND | **STAR GND** point at the battery negative (the single logic-reference wire) |
| **[32]** | GPIO12 (hardware PWM) | IBT-2 #1 (FRONT) pin 1 **RPWM** |
| **[33]** | GPIO13 (hardware PWM) | IBT-2 #2 (REAR) pin 1 **RPWM** |

Unused (leave free): all other pins. The QNX serial debug console on the Pi 5 is the separate 3-pin JST connector, not GPIO14/15.

The motor pins can be remapped without recompiling (`pin_front_rpwm = …` in a `-c` config file). Keep RPWM on GPIO 12 and 13: they are hardware-PWM capable on separate channels.

---

## 3. RD-03D radar ↔ Pi (UART, 256000 baud 8N1)

| RD-03D pin | Pi pin | Note |
|---|---|---|
| 5V | [2] 5 V | radar needs 5 V (≥ 200 mA peak) |
| GND | [6] GND | |
| TX | [10] GPIO15 RXD0 | **crossed**: radar TX → Pi RX (3.3 V level) |
| RX | [8] GPIO14 TXD0 | **crossed**: Pi TX → radar RX |

- QNX device: **`/dev/ser11`** (RP1 UART0 on GPIO14/15), started by `scripts/start_uart.sh` if it does not already exist.
- **Mounting:** front centre of the car, antenna face pointing forward, vertical, about 10–20 cm above the floor. Measure its height and keep it level. The code assumes **+x = right, +y = forward** from the radar.
- The radar is Doppler-based: it only reports objects that **move relative to it**.

---

## 4. MPU6050 (GY-521) ↔ Pi (I²C1, address 0x68)

| MPU6050 pin | Pi pin | Note |
|---|---|---|
| VCC | [1] 3.3 V | 3.3 V keeps the module's SDA/SCL pull-ups at 3.3 V |
| GND | [9] GND | |
| SCL | [5] GPIO3 SCL1 | |
| SDA | [3] GPIO2 SDA1 | |
| AD0 | GND (jumper to the module's GND pin) | address **0x68** |
| INT, XDA, XCL | not connected | |

- QNX device: **`/dev/i2c1`**.
- **Mounting:** screw or hot-glue it flat and rigid near the car's centre (not on a vibrating bracket). Note which chip axis points **forward**. The default is chip **X forward** (`imu_forward_axis = 0`, `imu_forward_sign = 1`). Check it with `aeb -m imu`: push the car forward and `a_fwd` must go **positive**.
- Keep the car **still for about 2 s after start-up** while the IMU calibrates.

---

## 5. BTS7960 / IBT-2 control header ↔ Pi

IBT-2 8-pin header (confirmed pin order): **1 RPWM · 2 LPWM · 3 R_EN · 4 L_EN · 5 R_IS · 6 L_IS · 7 VCC · 8 GND**

### BTS7960 #1 — FRONT axle (M1 front-left + M3 front-right)

| IBT-2 #1 pin | Connects to |
|---|---|
| 1 RPWM | Pi [32] GPIO12 · 10 kΩ to IBT-2 #1 pin 8 |
| 2 LPWM | Pi [15] GPIO22 |
| 3 R_EN | Pi [16] GPIO23 · **jumper to pin 4** · 10 kΩ to IBT-2 #1 pin 8 |
| 4 L_EN | jumper from pin 3 |
| 5 R_IS | **NOT CONNECTED** |
| 6 L_IS | **NOT CONNECTED** |
| 7 VCC | Pi [17] 3.3 V (and onward to IBT-2 #2 pin 7) |
| 8 GND | only the pull-down resistors; **no wire to the Pi** (it is internally B−) |

### BTS7960 #2 — REAR axle (M2 rear-left + M4 rear-right)

| IBT-2 #2 pin | Connects to |
|---|---|
| 1 RPWM | Pi [33] GPIO13 · 10 kΩ to IBT-2 #2 pin 8 |
| 2 LPWM | Pi [13] GPIO27 |
| 3 R_EN | Pi [18] GPIO24 · **jumper to pin 4** · 10 kΩ to IBT-2 #2 pin 8 |
| 4 L_EN | jumper from pin 3 |
| 5 R_IS | **NOT CONNECTED** |
| 6 L_IS | **NOT CONNECTED** |
| 7 VCC | from IBT-2 #1 pin 7 (= Pi 3.3 V) |
| 8 GND | only the pull-down resistors; **no wire to the Pi** |

**Why VCC = 3.3 V (not 5 V):** VCC only powers the module's logic input buffer and LED. At 3.3 V every logic level on the header stays at or below 3.3 V, so nothing can push 5 V back into the Pi. The 3.3 V Pi signals also become full-swing logic levels for the buffer. The BTS7960 inputs accept 3.3 V logic, and the module's own spec lists a "3.3–5 V" control level.
*Fallback only if a module does not respond at 3.3 V:* move that module's VCC to Pi 5 V [4]. All other connections stay the same; never connect R_IS/L_IS.

**What the Pi signals do** (implemented in `motor_bts7960.c`):

| Command | EN (pins 3+4) | RPWM | LPWM | Result |
|---|---|---|---|---|
| DISABLE (boot, exit, motor fault) | 0 | 0 | 0 | both half-bridges OFF → coast |
| DRIVE (normal / partial / strong) | 1 | PWM duty | 0 | forward; PWM off-phase shorts the motor (braking component) |
| BRAKE (emergency, stopped, faults) | 1 | 0 | 0 | both low-side switches ON → motor shorted → dynamic brake |

---

## 6. Pull-down resistors (strongly recommended)

Solder or plug them **at the driver's header**:

| Resistor | From | To |
|---|---|---|
| R1 10 kΩ | IBT-2 #1 pin 3 (R_EN, jumpered to L_EN) | IBT-2 #1 pin 8 GND |
| R2 10 kΩ | IBT-2 #1 pin 1 (RPWM) | IBT-2 #1 pin 8 GND |
| R3 10 kΩ | IBT-2 #2 pin 3 (R_EN, jumpered to L_EN) | IBT-2 #2 pin 8 GND |
| R4 10 kΩ | IBT-2 #2 pin 1 (RPWM) | IBT-2 #2 pin 8 GND |

They hold the drivers **OFF** while the Pi boots (GPIOs are still inputs), if a Dupont wire falls off, or if the Pi is off while the master switch is on. The software also drives every motor pin LOW at start-up, and the external `aeb_guard` process covers a crash. The resistors are the last, hardware-level layer.

---

## 7. Motor power, master switch and the single-point ground

```
 12 V BATTERY (+) ──[FUSE]──[MASTER SWITCH]──► "+12V SWITCHED" block ──┬──► IBT-2 #1  B+
                   (close to                                           └──► IBT-2 #2  B+
                    the battery)
 12 V BATTERY (−) ─────────────────────────► "STAR GND" block ──────────┬──► IBT-2 #1  B−   (thick)
                                                                        ├──► IBT-2 #2  B−   (thick)
                                                                        └──► Pi [30] GND    (thin, logic reference only)
```

| From | To | Wire |
|---|---|---|
| Battery + | fuse holder (within 10 cm of the battery) | 1.5 mm² / 16 AWG |
| Fuse holder | master switch, terminal 1 | 1.5 mm² |
| Master switch, terminal 2 | "+12 V SWITCHED" block | 1.5 mm² |
| "+12 V SWITCHED" block | IBT-2 #1 **B+** | 1.0–1.5 mm² |
| "+12 V SWITCHED" block | IBT-2 #2 **B+** | 1.0–1.5 mm² |
| Battery − | "STAR GND" block | 1.5 mm² |
| "STAR GND" block | IBT-2 #1 **B−** | 1.0–1.5 mm² |
| "STAR GND" block | IBT-2 #2 **B−** | 1.0–1.5 mm² |
| "STAR GND" block | Pi **[30] GND** | thin jumper, logic reference only |

- **Before wiring, check with a multimeter (continuity mode)** that IBT-2 header pin 8 (GND) beeps to that module's **B−** terminal on both modules. That internal link is how the drivers' logic is referenced to the Pi through the star point.
- The **master switch cuts motor power only**. The Pi, radar and IMU stay alive, so the software can be tested with the switch OFF (wheels never move), and the switch is your hardware emergency stop.
- Never route motor current through a breadboard.

---

## 8. Motors ↔ drivers — one driver per AXLE

Driver #1 drives the **front** axle (M1 + M3). Driver #2 drives the **rear** axle (M2 + M4). The two motors of an axle are in parallel on that driver's M+/M− terminals.

**Important — mirrored motors:** the left and right motors of an axle face opposite directions. If both were wired the same way, the left wheel would roll forward and the right wheel backward, and the car would spin on the spot. So the **right-hand motor of each axle is wired crossed**:

| Motor | Motor terminal A (the "+" / red-wire terminal) | Motor terminal B (the "−" / black-wire terminal) |
|---|---|---|
| M1 front-left | IBT-2 #1 **M+** | IBT-2 #1 **M−** |
| M3 front-right | IBT-2 #1 **M−** ← crossed | IBT-2 #1 **M+** ← crossed |
| M2 rear-left | IBT-2 #2 **M+** | IBT-2 #2 **M−** |
| M4 rear-right | IBT-2 #2 **M−** ← crossed | IBT-2 #2 **M+** ← crossed |

(This assumes four identical motors mounted mirror-image, which is how a standard 4WD chassis is built.)

**Verify with the motor test** (`aeb -m motor -y`, wheels OFF the ground):

| What you see in the FRONT step (or the REAR step) | Fix |
|---|---|
| Both wheels of that axle turn forward | correct |
| One wheel forward, one backward | swap the two wires of the **backward** motor only |
| Both wheels backward | swap the two wires at that **driver's M+ and M−** terminals |

Do not drive on the floor until both axles pass. Two motors on one axle fighting each other load both like a stall.

**Why the axle split is good for braking:** if one driver or its wiring fails, the other axle still brakes the left and right wheels **equally**, so the car slows straight without pulling to one side. This is the same idea as a car's dual-circuit brakes. (A left/right split would brake one side only and make the car yaw.) The trade-off is that a front/rear split cannot steer by skid-steering, which this project does not need.

**Practical wiring:** put two wires into each M+ / M− screw terminal (one per motor), or run one pair to a small terminal block at the axle centre and branch from there. Use 1.0 mm² (18 AWG) or thicker.

- Recommended: one 100 nF ceramic capacitor soldered **directly across each motor's two terminals** (EMI suppression). No external flyback diodes are needed; the BTS7960 half-bridges freewheel internally.
- Recommended: one 470–1000 µF / 25 V electrolytic across **B+ (+) / B− (−)** at each driver to absorb motor transients.

---

## 9. Motor current: measure it (it is still unknown)

The motor rated current and stall current have **not** been measured, so the fuse and switch cannot be sized from data yet. Do this before the first full-power run:

1. **Stall-current estimate:** with the motor disconnected, measure its winding resistance R with a multimeter (turn the shaft slowly and take the lowest stable reading). Then `I_stall ≈ 12 V / R` for each motor. Example: R = 4 Ω gives about 3 A.
2. **Running current:** put the multimeter (10 A range) in series with one driver's B+ wire. Run `aeb -m motor -y` and note the current with both wheels of that side on the ground.
3. **Per driver:** `I_driver = I_left_motor + I_right_motor` of that axle. Worst case (both stalled) is `2 × I_stall`, and a start from rest briefly reaches about `duty × I_stall` per motor.
4. **Fuse:** slow-blow, above the measured running total of all four motors with margin (about 2×), and below the rating of the 1.5 mm² wire (about 15 A). With typical small 12 V geared motors this usually lands at 7.5–10 A, but **use your measurement**.
5. **Master switch:** DC current rating ≥ fuse rating (switch ratings printed for AC 250 V do **not** apply to 12 V DC loads).
6. **Driver thermal check:** after a 2-minute run, touch-test each BTS7960 heatsink. If it is too hot to hold, reduce `cruise_duty` or improve cooling.
7. **6 V motors on 12 V:** if your motors are 3–6 V types, set `motor_max_duty = 50` (average voltage ≤ 6 V) in the config file.

---

## 10. Power-up / power-down order

**Power-up**
1. Master switch **OFF**, battery connected.
2. Power the Pi (USB-C) and wait for QNX to boot.
3. `./start_uart.sh`, then `./aeb_guard &`, then `./aeb -m full` (motors are DISABLED at start).
4. Keep the car still for about 2 s (IMU calibration).
5. Master switch **ON**.
6. Type `go`.

**Power-down**
1. `stop`, then `quit` (outputs go to DISABLE).
2. Master switch **OFF**.
3. Shut down or unplug the Pi.

**Emergency:** flip the master switch OFF (hardware), or type `estop` (software latch).

---

## 11. Quick electrical checks before first power

| Check (battery disconnected, Pi off) | Expected |
|---|---|
| Continuity: IBT-2 #1 pin 8 ↔ #1 B− | beep |
| Continuity: IBT-2 #2 pin 8 ↔ #2 B− | beep |
| Continuity: Pi [30] ↔ STAR GND | beep |
| Continuity: battery + ↔ Pi [2], [4], [1], [17] | **NO** beep |
| Continuity: R_IS / L_IS ↔ anything on the Pi | **NO** beep |
| Resistance: IBT-2 #1 pin 3 ↔ pin 8 | ≈ 10 kΩ |
| Fuse and master switch are both in the **+** line | yes |

**With the Pi on and the master switch OFF:** measure IBT-2 pin 7 to pin 8: about 3.3 V. Measure EN (pin 3) to pin 8: 0 V until `go`.

## 12. Camera Module 3 (IMX708) — optional, for object classification

| From | To | Notes |
|---|---|---|
| Camera Module 3 (15-pin FPC) | Pi 5 **CAM/DISP 0** (22-pin FPC) | Use the **Raspberry Pi 5 camera cable (22-pin ↔ 15-pin)**. The cable that comes with the camera does not fit the Pi 5. Connect it with the **power off**. Contacts face the connector contacts, and the latch must be closed at both ends. |

- QNX uses this camera as **unit 3** (`/etc/config/sensor/sensor_rpi5.conf`, QNX 8.0.5 QSTI). Its I²C control bus is `/dev/i2c6`, which is internal to the CSI connector. No GPIO header pins are used, so nothing changes in §2.
- Mount it facing forward, next to the radar, level, and with nothing blocking the view. The software assumes a 66° horizontal field of view (`-F 102` for the Wide version).
- Steps, commands and tests: `docs/VISION.md`.
