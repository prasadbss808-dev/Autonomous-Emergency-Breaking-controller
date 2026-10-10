#ifndef CAMERA_IF_H
#define CAMERA_IF_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    bool valid;
    uint64_t t_ns;
    int class_id;
    float confidence;
    float bearing_deg;
    float area_frac;
    float fps;
    float infer_ms;
    char label[16];
} camera_det_t;
int camera_init(void);
bool camera_get_latest_detection(camera_det_t *out);
bool camera_connected(void);
#endif
