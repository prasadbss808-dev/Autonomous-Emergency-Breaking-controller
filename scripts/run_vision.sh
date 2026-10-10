#!/bin/sh
# Start the camera object-detection process (IMX708 + OpenCV MobileNet-SSD).
# Runs at low priority on CPUs 2-3 and publishes to shared memory /aeb_vision
# (read by the AEB and the dashboard). Prints "OBJECT DETECTED: PERSON" lines.
#   ./run_vision.sh                     camera (unit 3 if present, else unit 1)
#   ./run_vision.sh -o vision_last.jpg  also save an annotated frame every 2 s
#   ./run_vision.sh -i models/street.jpg -o out.jpg   offline test without the camera
#   ./run_vision.sh -i models/person.jpg -i models/street.jpg -r   camera stand-in (replay)
#   ./run_vision.sh -L                  list camera units
cd "$(dirname "$0")"
LD_LIBRARY_PATH=$PWD/opencv_libs:/data/home/qnxuser/opencv/libs:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH
UNIT=1
[ -e /dev/sensor/camera3 ] && UNIT=3
case " $* " in
    *" -u "*|*" -i "*|*" -L "*) exec ./aeb_vision -m ./models "$@" ;;
    *)                          exec ./aeb_vision -m ./models -u $UNIT "$@" ;;
esac
