# Real-time dashboard

`aeb_dash` is a small web server on the Pi, port 8080. Open **http://<pi-ip>:8080** in any
browser on the same network: the laptop over Ethernet, or a phone or laptop over Wi-Fi.
It needs no internet and no extra software, and it works on a phone screen.

```
aeb (RT threads) --log ring--> logger thread (prio 10) --seqlock--> /aeb_telemetry  --+
aeb_vision (prio 8) ---------------------------------------------> /aeb_vision     --+--> aeb_dash (prio 9) --HTTP--> browser
                                                                    /aeb_vision_frame -+        |
aeb main thread (prio 10) <----------------------- /aeb_control <-----------------------------+  (GO / STOP / E-STOP)
```

- The real-time threads (brake, prediction, PWM, radar, watchdog) never touch the dashboard path.
  The AEB logger thread publishes a snapshot about 20 times per second.
- All links are wait-free shared memory. A slow network, a closed browser or a crashed `aeb_dash`
  cannot delay braking.
- `aeb_dash` runs at priority 9, below every AEB thread.

## What it shows (updates about 10 times per second)

| Panel | Content |
|---|---|
| AEB state | NORMAL / WARNING / PARTIAL / STRONG / EMERGENCY / STOPPED (colour coded), risk, motor command and duty |
| Distance, closing speed, TTC | the radar threat in the driving corridor; TTC is coloured by threshold; d_safe for P/S/F |
| Brake command latency | decision → outputs for the last cycle and the worst since start (deadline 50 000 µs) |
| Radar bird's-eye view | all RD-03D tracks: threat (red), in path (amber), beside the path (grey), closing-speed vectors, driving corridor, stop line, d_safe |
| Camera | live annotated frame from the IMX708 camera and the list `OBJECT DETECTED: CAR / PERSON …` (AHEAD = inside the ±20° corridor) |
| Timeline | distance and TTC over the last 30 s, with a state colour band |
| System health | radar / IMU / motor / system / guard / camera, decoded fault bits, radar frame period |
| Events | the AEB event log (state changes, emergency brakes, `OBJECT DETECTED`, camera confirmations) |
| Controls | **GO** (asks for confirmation), **STOP**, **E-STOP**, clear impact latch. The keyboard space bar = E-STOP. |

GO is only accepted when the AEB logic allows it: radar healthy and path clear.
The master switch is still the hardware stop.

## Start

```sh
./start_all.sh          # everything with a terminal (Ctrl-C = safe stop)
./run_dash.sh           # only the dashboard server (if the AEB is started some other way)
```

`start_all.sh` prints the URL. To find the Pi's address yourself: `ifconfig | grep "inet "`.

## API (for scripts and testing)

- `GET /api/state` returns a JSON snapshot `{aeb:{…}, vision:{…}}`. `aeb` is `null` if the AEB is not running.
- `GET /api/frame.jpg` returns the latest annotated camera frame, or 404 if the camera is off or stale.
- `GET /api/cmd?c=go|stop|estop|clear` sends a command to the AEB.

## Troubleshooting

| Symptom | Fix |
|---|---|
| Page does not load | `pidin -p aeb_dash`; check the IP; try `curl http://localhost:8080/api/state` on the Pi |
| Badge says AEB NOT RUNNING | start `./aeb …` (the dashboard keeps running and reconnects automatically) |
| "camera not running" | start `./run_vision.sh`; with colour bars, run `./camera_setup.sh real` (switches off the simulator) |
| Badge says OFFLINE | the browser lost the network link to the Pi |
