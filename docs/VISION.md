# Camera object detection on the QNX Pi (IMX708 + OpenCV)

The Raspberry Pi Camera Module 3 (IMX708) captures frames, and OpenCV runs the
object detector. Both run **on the QNX Raspberry Pi 5 itself**.

```
Camera Module 3 ──CSI (CAM/DISP 0)──► QNX Sensor Framework (sensor service, unit 3, NV12)
                                          │ libcamapi viewfinder callback
                                          ▼
                     aeb_vision  (C++, priority 8, CPUs 2-3, 2 OpenCV threads)
                     NV12 → downscale 640 px → BGR → MobileNet-SSD (cv::dnn, 300×300)
                     → largest object inside ±20° corridor → label, confidence, bearing
                                          │ POSIX shared memory "/aeb_vision" (seqlock, wait-free)
                                          ▼
                     aeb (vehicle thread, RR 30) → camera_if.c → events in the log:
                     "camera: person ahead", "EMERGENCY_BRAKE: camera CONFIRMS person …"
```

**Safety role:** the camera is complementary only. It confirms and classifies
objects. The radar stays the only ranging sensor and the only thing that
triggers braking:

- The camera can never block, delay or cancel a brake.
- If `aeb_vision` crashes or stalls, the AEB logs `camera … not connected - radar-only AEB` and carries on unchanged.
- `aeb_vision` runs at priority 8 on CPUs 2–3, below every AEB thread (lowest AEB thread = 10). The brake and prediction threads run on CPU0 and CPU1.

These facts were verified from QNX's own material (QNX 8.0.5 QSTI image for Pi 5, `qnx/codelabs`, `qnx-ports/mediapipe` `qnx_defs.cc`):

- Camera Module 3 on the Pi 5 is supported by the sensor service with `/etc/config/sensor/sensor_rpi5.conf`.
- **Camera unit 3** is the Camera Module 3 on **DISP0**. It delivers NV12 at 2304×1296@30 and uses I²C `/dev/i2c6`.
- Frames are captured with `libcamapi`: `camera_open` → `camera_start_viewfinder`.
- OpenCV `VideoCapture` does **not** work on QNX (no QNX camera backend in qnx-ports OpenCV 4.9.0). That is why `aeb_vision` uses `libcamapi` directly.

---

## Output

For every class that appears in view (repeated every 2 s while it stays in view), `aeb_vision` prints:
```
OBJECT DETECTED: PERSON      (conf 0.96, bearing +7 deg, AHEAD)
OBJECT DETECTED: CAR         (conf 0.99, bearing -25 deg)
```
The AEB log shows `OBJECT DETECTED: PERSON (camera, …, ahead)` for the object in the driving corridor.
The dashboard (`docs/DASHBOARD.md`) shows the live annotated frame and the full list.

## Step 1: Hardware

1. **Power the Pi off.**
2. Connect the Camera Module 3 to the **CAM/DISP 0** connector, using the **Raspberry Pi 5 camera cable (22-pin to 15-pin)**. The cable shipped with the camera is 15-pin at both ends and does not fit the Pi 5.
   - Lift the connector latch.
   - Insert the cable fully and straight, with the exposed contacts facing the contacts inside the connector.
   - Close the latch. Do the same at the camera end.
3. Mount the camera facing forward, next to the radar, and level. A good starting height is 10–20 cm above the floor.
4. The camera's I²C bus is `/dev/i2c6`. It is **separate** from the MPU6050 bus (`/dev/i2c1`, GPIO2/3), so there is no conflict.

## Step 2: Check the QNX camera stack (on the Pi, as root)

```sh
cd /data/home/qnxuser/aeb
./check_camera.sh 3            # after "make deploy-vision", or copy the script over first
```

Or check by hand:

```sh
use -i /proc/boot/procnto-smp-instr | grep TAGID   # need QNXOS_805 or newer
pidin ar | grep '[s]ensor'                          # sensor -U 521:521 ... -c /etc/config/sensor/sensor_rpi5.conf
ls -l /dev/sensor                                   # camera3 must exist
slog2info -b sensor_service | grep -E "Camera 3"    # "Camera 3: ... resolution 2304 x 1296"
camera_example3_viewfinder -u 3                     # live picture on an HDMI monitor (Ctrl-C to stop)
```

If any of these checks fail:

- **No `sensor` process, or no `/dev/sensor/camera3`:**
  - Your image is older than QNX 8.0.5 or has no camera support.
  - Install the **QNX 8.0.5 QSTI image for Raspberry Pi 5** from QNX Software Center (`com.qnx.qnx800.quickstart.rpi5`, 0.5.x) and flash it.
  - Reflashing erases the SD card, so copy `aeb.conf` and your logs off first. Afterwards, redo `start_uart.sh` and `gpio-rp1 set 2,3 a3`.
- **Older image with apk:** packages can be (re)installed with `sudo apk add qnx-sensor-framework qnx-sensor-framework-utils qnx-sensor-framework-example-configs qnx-sf-camera-imx708 qnx-sf-platform-broadcom-rpi5 qnx-rpi-camera-ipa`. These names come from the QNX codelab for Pi 5 cameras.
- **"Camera 3" missing in `slog2info`:** reseat the ribbon cable at both ends. It is usually inserted the wrong way round or not fully.

## Step 3: Get OpenCV (and the camera header/library) for QNX

Use **one** of the two paths below.

### Path A: prebuilt packages on the Pi (fastest, needs an image with `apk`)

```sh
# on the Pi
which apk && cat /etc/apk/repositories
sudo apk update
sudo apk add opencv-dev qnx-sf-base-dev      # OpenCV libs+headers, camera_api.h
find /usr/include -path '*opencv2/dnn.hpp'   # note the include dir (e.g. /usr/include/opencv4)
ls /usr/lib/libopencv_dnn.so* /usr/lib/libcamapi.so*
which c++ clang++ q++ 2>/dev/null            # is there a compiler on the Pi?
```

**A1: there is a compiler on the Pi.** Build directly on the Pi:

```sh
# on the Pi, in the project folder (copy vision/ over first)
c++ -std=gnu++17 -O2 -Ivision -I/usr/include/opencv4 vision/aeb_vision.cpp -o aeb_vision \
    -lopencv_dnn -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lcamapi
```

**A2: no compiler on the Pi.** Copy the headers and libraries to your PC and cross-compile there:

```sh
# on the PC (Linux / WSL), QNX SDP 8.0 environment sourced
PI=qnxuser@<pi-ip>
R=~/qnx_workspace/pi_sysroot
mkdir -p $R/usr/include $R/aarch64le/usr/lib
scp -r $PI:/usr/include/opencv4 $PI:/usr/include/camera $R/usr/include/
scp "$PI:/usr/lib/libopencv_*.so*" "$PI:/usr/lib/libcamapi.so*" $R/aarch64le/usr/lib/
cd qnx_aeb
make vision VISION_SYSROOT=$R          # -> build/aeb_vision
```

With Path A, the OpenCV libraries are already installed on the Pi, so nothing else needs copying.

### Path B: build OpenCV from qnx-ports on the PC (works with any QNX 8.0 image)

This follows `qnx-ports/build-files/ports/opencv/README.md`, which builds OpenCV 4.9.0 (`qnx-4.9.0` branch).

- qnx-ports needs a **Linux host**. Use Ubuntu, WSL2 Ubuntu, or the qnx-ports Docker container.
- Install the `com.qnx.qnx800.target.screen.img_codecs` package in your SDP, using QNX Software Center.

```sh
mkdir -p ~/qnx_workspace && cd ~/qnx_workspace
git clone https://github.com/qnx-ports/build-files.git
git clone https://github.com/qnx-ports/opencv.git
source ~/qnx800/qnxsdp-env.sh

# We only need C++ core/imgproc/imgcodecs/dnn: skip the Python bindings (no
# numpy / python3.11 needed) and the other modules -> much faster build.
sed -i 's/-DBUILD_opencv_python3=ON/-DBUILD_opencv_python3=OFF -DBUILD_LIST=core,imgproc,imgcodecs,dnn/' \
    build-files/ports/opencv/common.mk

QNX_PROJECT_ROOT="$(pwd)/opencv" make -C build-files/ports/opencv \
    INSTALL_ROOT_nto=$HOME/qnx_workspace/staging USE_INSTALL_ROOT=true install -j$(nproc)

find ~/qnx_workspace/staging -name 'libopencv_dnn.so*'      # aarch64le/usr/local/lib/...
```

If you want the full upstream build, including Python and tests, follow the README exactly. It also builds `numpy` and `muslflt`.

**Camera header and library for Path B:**

```sh
ls $QNX_TARGET/usr/include/camera/camera_api.h $QNX_TARGET/aarch64le/usr/lib/libcamapi.so*
```

If these files are missing from your SDP, either:

- install the Sensor Framework / camera package in QNX Software Center (search for "sensor"), or
- copy them from the Pi. `libcamapi.so` is in `/usr/lib` on any image whose camera works. `camera_api.h` comes with `qnx-sf-base-dev`.

  ```sh
  R=~/qnx_workspace/pi_sysroot; mkdir -p $R/usr/include $R/aarch64le/usr/lib
  scp -r $PI:/usr/include/camera $R/usr/include/
  scp "$PI:/usr/lib/libcamapi.so*" $R/aarch64le/usr/lib/
  ```

**Build, then copy the OpenCV libraries to the Pi:**

```sh
cd qnx_aeb
make vision-info OPENCV_ROOT=~/qnx_workspace/staging     # shows the dirs it found
make vision      OPENCV_ROOT=~/qnx_workspace/staging     # -> build/aeb_vision
make deploy-vision OPENCV_ROOT=~/qnx_workspace/staging TARGET=qnxuser@<pi-ip>
scp ~/qnx_workspace/staging/aarch64le/usr/local/lib/libopencv_{core,imgproc,imgcodecs,dnn}.so* \
    qnxuser@<pi-ip>:/data/home/qnxuser/aeb/opencv_libs/
```

`run_vision.sh` adds `./opencv_libs` to `LD_LIBRARY_PATH`.

## Step 4: Deploy the program and the model

```sh
make deploy-vision TARGET=qnxuser@<pi-ip>    # (+ OPENCV_ROOT=… or VISION_SYSROOT=… as above)
```

This copies the following to `/data/home/qnxuser/aeb/` on the Pi:

- `aeb_vision`, `run_vision.sh` and `check_camera.sh`
- `models/deploy.prototxt` and `models/mobilenet_iter_73000.caffemodel` (MobileNet-SSD, 20 VOC classes, MIT licence, 23 MB). The model files are already inside this project, so no download is needed.
- two test pictures, `models/person.jpg` and `models/street.jpg`

## Step 5: Test on the Pi (one step at a time)

```sh
cd /data/home/qnxuser/aeb

# 5a  OpenCV + model only, no camera
./run_vision.sh -i models/street.jpg -o street_out.jpg
#   IMG models/street.jpg (500x375): ... AHEAD: horse conf 1.00 bearing +9.4 deg ...
#   (on the PC: scp qnxuser@<pi-ip>:/data/home/qnxuser/aeb/street_out.jpg . and open it)

# 5b  list the camera units
./run_vision.sh -L

# 5c  live camera detection (stand in front of the camera)
./run_vision.sh -o vision_last.jpg
#   VIS: unit 3 viewfinder format = ... (NV12 - OK)
#   VIS: first frame: source 2304x1296 NV12 -> processed 640x360 BGR
#   VIS frame  42 |  9.8 fps | infer  71.0 ms | 1 det | AHEAD: person conf 0.93 bearing -2.1 deg size 0.214
#   Ctrl-C to stop; vision_last.jpg is refreshed every 2 s (red boxes = inside the corridor)

# 5d  together with the AEB (two terminals / ssh sessions)
#   terminal 1:
./run_vision.sh -q &
#   terminal 2 (as before; root):
./start_uart.sh
./aeb_guard &
./aeb -m full -c aeb.conf
#   expected log lines:
#   INFO camera (aeb_vision) connected (inference 70 ms) - complementary classification only
#   INFO camera: person ahead (conf 0.93, bearing -2 deg)
#   WARN EMERGENCY BRAKE applied ... decision->outputs 25 us (deadline 50000 us)
#   INFO EMERGENCY_BRAKE: camera CONFIRMS person (conf 0.93, bearing -2 deg), radar range 0.98 m
#   then: slay aeb_vision   ->  "camera (aeb_vision) not connected - radar-only AEB", AEB keeps working

# 5e  integration test WITHOUT a camera (replays the pictures as if they were camera frames)
./run_vision.sh -i models/person.jpg -i models/street.jpg -r -q &
```

## Step 6: Show that the vision load doesn't hurt real-time braking

```sh
pidin -p aeb_vision          # threads at priority 8
pidin -p aeb                 # AEB threads 63/62/58/57/50/30/10, unchanged
hogs -i 5                    # CPU use per process
```

The proof is the `decision->outputs NN us (deadline 50000 us)` line in the AEB log. It should stay in the tens of µs while `aeb_vision` is running.

## Options (`./aeb_vision -h`)

| Option | Meaning | Default |
|---|---|---|
| `-u N` | camera unit | 3 |
| `-t TH` | confidence threshold | 0.50 |
| `-F DEG` | horizontal FOV (Camera Module 3 = 66, Wide = 102) | 66 |
| `-k DEG` | corridor half-angle counted as "ahead" | 20 |
| `-X N` | downscale frames wider than N | 640 |
| `-j N` | OpenCV threads | 2 |
| `-R HEX` / `-P N` | CPU runmask / priority | 0xC / 8 |
| `-o FILE -s SEC` | save annotated frame every SEC s | off / 2 |
| `-i FILE [-r]` | offline image test / replay as camera stand-in | – |

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `camera_open(unit 3) failed` | `ls /dev/sensor`. Check the unit list (`-L`) and that `sensor` is running. Run as root or as a user in the `sensor` group. |
| `NOT NV12` | This program converts NV12 only. Check `default_video_format = nv12` in the sensor config. |
| `no usable frame for N s (callbacks 0)` | The camera is not streaming. Reseat the ribbon cable and check `slog2info -b sensor_service`. |
| `error while loading shared libraries: libopencv_…` | The libraries are not in `./opencv_libs` or `/usr/lib`. Start the program through `run_vision.sh`. |
| `cannot load model` | `models/deploy.prototxt` and `models/mobilenet_iter_73000.caffemodel` must be next to the program. |
| AEB says `camera … not connected` | `aeb_vision` is not running, or it cannot create `/dev/shmem/aeb_vision`: `ls -l /dev/shmem/aeb_vision`. |
| `weights of 'conv0' are invalid … byte-swap work-around` | Expected with the qnx-ports OpenCV 4.9 build on QNX. Its internal protobuf reads float weights with their bytes reversed (verified on the Pi: 999.98236 was read as -3.667e19). `aeb_vision` detects this and pre-reverses the floats in the `.caffemodel` buffer before loading, then checks the weights are sane. The proper fix is to rebuild OpenCV with the correct little-endian setting for protobuf. |
| Colours look wrong in the saved image | Detection is barely affected. If needed, the conversion is `COLOR_YUV2BGR_NV12` in `cam_vf_cb()`. |

## Files

- `vision/aeb_vision.cpp` — capture, conversion, detection, shared-memory publisher
- `vision/vision_shm.h` — shared record and seqlock, used by both sides
- `src/camera_if.c` — AEB reader (wait-free, retries the attach once per second)
- `src/vehicle_state.c` — camera connect/loss events, classification events, "camera CONFIRMS" at brake onset
- `scripts/run_vision.sh`, `scripts/check_camera.sh`
- `vision/models/` — MobileNet-SSD (chuanqi305/MobileNet-SSD, MIT licence)
- `vision/test_images/` — two sample pictures for the offline test
