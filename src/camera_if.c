#include "camera_if.h"
#include "../vision/vision_shm.h"
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#define CAM_HEARTBEAT_NS 1000000000ull
#define CAM_DET_MAX_AGE_NS 300000000ull
#define CAM_RETRY_NS 1000000000ull
static const vision_shm_t *g_shm;
static uint64_t g_last_try_ns;
static bool g_connected;
static camera_det_t g_last;
static uint64_t cam_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}
static void cam_try_attach(uint64_t now)
{
    if (g_shm) return;
    if (g_last_try_ns != 0 && now - g_last_try_ns < CAM_RETRY_NS) return;
    g_last_try_ns = now;
    int fd = shm_open(VISION_SHM_NAME, O_RDONLY, 0);
    if (fd < 0) return;
    void *p = mmap(NULL, sizeof(vision_shm_t), PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED) return;
    g_shm = (const vision_shm_t *)p;
}
int camera_init(void)
{
    memset(&g_last, 0, sizeof g_last);
    g_last.class_id = -1;
    g_connected = false;
    g_shm = NULL;
    g_last_try_ns = 0;
    cam_try_attach(cam_now_ns());
    return g_shm ? 0 : -1;
}
bool camera_connected(void)
{
    return g_connected;
}
bool camera_get_latest_detection(camera_det_t *out)
{
    uint64_t now = cam_now_ns();
    cam_try_attach(now);
    if (g_shm) {
        vision_shm_t s;
        if (vision_shm_read(g_shm, &s, 4) && s.magic == VISION_SHM_MAGIC && s.version == VISION_SHM_VERSION) {
            uint64_t age = (s.t_ns != 0 && now >= s.t_ns) ? now - s.t_ns : UINT64_MAX;
            g_connected = (s.t_ns != 0) && (age < CAM_HEARTBEAT_NS);
            g_last.t_ns = s.t_ns;
            g_last.class_id = s.class_id;
            g_last.confidence = s.confidence;
            g_last.bearing_deg = s.bearing_deg;
            g_last.area_frac = s.area_frac;
            g_last.fps = s.fps;
            g_last.infer_ms = s.infer_ms;
            memcpy(g_last.label, s.label, sizeof g_last.label);
            g_last.label[sizeof g_last.label - 1] = '\0';
            g_last.valid = g_connected && s.class_id > 0 && age < CAM_DET_MAX_AGE_NS;
        } else if (g_last.t_ns == 0 || now - g_last.t_ns >= CAM_HEARTBEAT_NS) {
            g_connected = false;
            g_last.valid = false;
        }
    } else {
        g_connected = false;
        g_last.valid = false;
    }
    *out = g_last;
    return g_last.valid;
}
