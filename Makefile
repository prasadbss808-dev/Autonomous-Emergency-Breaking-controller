# =============================================================================
#  QNX AEB controller - Makefile
#
#  Cross-compile for Raspberry Pi 5 (QNX SDP 8.0, aarch64le):
#     source ~/qnx800/qnxsdp-env.sh        # Linux/macOS host   (Windows: qnxsdp-env.bat)
#     make                                  # -> build/aeb  build/aeb_guard
#
#  Host unit/scenario tests (no QNX needed, any gcc/clang):
#     make host-test
# =============================================================================
PLATFORM   ?= aarch64le
QCC        ?= qcc -Vgcc_nto$(PLATFORM)
CSTD        = -std=gnu11
WARN        = -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
OPT        ?= -O2 -g
CFLAGS      = $(CSTD) $(WARN) $(OPT) -Isrc
LIBS        = -lslog2          # QNX slogger2 client library (pthread, timers, IPC are in libc)

SRCS        = $(wildcard src/*.c)
PURE_SRCS   = src/aeb_logic.c src/aeb_cycle.c src/radar_track.c src/rd03d_proto.c \
              src/imu_proc.c src/sim_model.c src/params.c src/selftest.c
GUARD_SRCS  = guard/aeb_guard.c src/gpio_rpi.c src/params.c

BUILD       = build

.PHONY: all clean host-test deploy vision vision-info deploy-vision dash deploy-all

all: $(BUILD)/aeb $(BUILD)/aeb_guard $(BUILD)/aeb_dash $(BUILD)/aeb_master

$(BUILD)/aeb: $(SRCS) src/*.h | $(BUILD)
	$(QCC) $(CFLAGS) -o $@ $(SRCS) $(LIBS)

$(BUILD)/aeb_guard: $(GUARD_SRCS) src/*.h | $(BUILD)
	$(QCC) $(CFLAGS) -o $@ $(GUARD_SRCS)

# ---- one launcher for everything (Momentics: Run As -> C/C++ QNX Application) ----
$(BUILD)/aeb_master: master/aeb_master.c | $(BUILD)
	$(QCC) $(CFLAGS) -o $@ master/aeb_master.c

# ---- dashboard web server (reads shared memory only; -lsocket = QNX io-sock) ----
dash: $(BUILD)/aeb_dash
$(BUILD)/aeb_dash: dash/aeb_dash.c src/telemetry.h vision/vision_shm.h | $(BUILD)
	$(QCC) $(CFLAGS) -Ivision -o $@ dash/aeb_dash.c -lsocket

$(BUILD):
	mkdir -p $(BUILD)

# ---- vision process: IMX708 camera (libcamapi) + OpenCV DNN, C++ ----------
#  make vision OPENCV_ROOT=<dir containing the qnx-ports OpenCV install>
#  Header/library dirs are searched automatically under OPENCV_ROOT,
#  VISION_SYSROOT (files copied from the Pi) and $QNX_TARGET; override
#  OPENCV_INC / OPENCV_LIB / CAMAPI_INC / CAMAPI_LIB if needed.
QCXX           ?= q++ -Vgcc_nto$(PLATFORM)_cxx
OPENCV_ROOT    ?= $(HOME)/qnx_workspace/staging
VISION_SYSROOT ?= $(HOME)/qnx_workspace/pi_sysroot
VSEARCH         = $(OPENCV_ROOT) $(VISION_SYSROOT) $(QNX_TARGET)
first_dir       = $(shell f=$$(find $(VSEARCH) $(1) 2>/dev/null | head -n 1); [ -n "$$f" ] && dirname "$$f")
OPENCV_INC     ?= $(patsubst %/opencv2,%,$(call first_dir,-path '*/opencv2/dnn.hpp'))
OPENCV_LIB     ?= $(call first_dir,-path '*aarch64le*' -name 'libopencv_dnn.so*')
CAMAPI_INC     ?= $(patsubst %/camera,%,$(call first_dir,-path '*/camera/camera_api.h'))
CAMAPI_LIB     ?= $(call first_dir,-path '*aarch64le*' -name 'libcamapi.so*')
VISION_CXXFLAGS = -std=gnu++17 -O2 -Wall -Wextra -Wno-unused-parameter -Ivision \
                  -I$(OPENCV_INC) -I$(CAMAPI_INC)
VISION_LIBS     = -L$(OPENCV_LIB) -L$(CAMAPI_LIB) -Wl,-rpath-link,$(OPENCV_LIB) \
                  -Wl,--allow-shlib-undefined \
                  -lopencv_dnn -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lcamapi

vision-info:
	@echo "OPENCV_INC = $(OPENCV_INC)"
	@echo "OPENCV_LIB = $(OPENCV_LIB)"
	@echo "CAMAPI_INC = $(CAMAPI_INC)"
	@echo "CAMAPI_LIB = $(CAMAPI_LIB)"

vision: $(BUILD)/aeb_vision

$(BUILD)/aeb_vision: vision/aeb_vision.cpp vision/vision_shm.h | $(BUILD)
	@if [ -z "$(OPENCV_INC)" ] || [ -z "$(OPENCV_LIB)" ]; then echo "ERROR: OpenCV not found - run 'make vision-info', set OPENCV_ROOT=..."; exit 1; fi
	@if [ -z "$(CAMAPI_INC)" ] || [ -z "$(CAMAPI_LIB)" ]; then echo "ERROR: camera_api.h / libcamapi.so not found - see docs/VISION.md step 3"; exit 1; fi
	$(QCXX) $(VISION_CXXFLAGS) -o $@ vision/aeb_vision.cpp $(VISION_LIBS)

deploy-vision: vision
	ssh $(TARGET) "mkdir -p $(TARGET_DIR)/models $(TARGET_DIR)/opencv_libs"
	scp $(BUILD)/aeb_vision scripts/run_vision.sh scripts/check_camera.sh $(TARGET):$(TARGET_DIR)/
	scp vision/models/deploy.prototxt vision/models/mobilenet_iter_73000.caffemodel $(TARGET):$(TARGET_DIR)/models/
	scp vision/test_images/*.jpg $(TARGET):$(TARGET_DIR)/models/

# ---- everything (AEB + guard + dashboard + camera detector) ----
deploy-all: deploy deploy-vision

# ---- host tests of the pure decision core (same code that runs on QNX) ----
HOSTCC ?= cc
host-test: | $(BUILD)
	$(HOSTCC) $(CSTD) $(WARN) -O2 -Isrc -o $(BUILD)/test_host tests/test_host.c $(PURE_SRCS)
	./$(BUILD)/test_host

# ---- copy to the Pi (QSTI default user/IP - edit TARGET) ----
TARGET     ?= qnxuser@192.168.1.50
TARGET_DIR ?= /data/home/qnxuser/aeb
deploy: all
	ssh $(TARGET) "mkdir -p $(TARGET_DIR)/www"
	scp $(BUILD)/aeb $(BUILD)/aeb_guard $(BUILD)/aeb_dash $(BUILD)/aeb_master scripts/*.sh aeb.conf $(TARGET):$(TARGET_DIR)/
	scp dash/www/index.html $(TARGET):$(TARGET_DIR)/www/

clean:
	rm -rf $(BUILD)
