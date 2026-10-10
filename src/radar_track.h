#ifndef RADAR_TRACK_H
#define RADAR_TRACK_H
#include "aeb_types.h"
#include "params.h"
#include "rd03d_proto.h"
typedef struct {
    uint8_t slot;
    float x_m, y_m, range_m;
    float v_radial_mps;
    bool doppler_valid;
    float res_m;
} radar_det_t;
typedef struct {
    radar_track_t pub;
    float last_raw_range_m;
} trk_int_t;
typedef struct {
    trk_int_t t[RADAR_MAX_TRACKS];
    int n;
    uint8_t next_id;
} tracker_t;
int radar_frame_to_detections(const rd03d_frame_t *f, const aeb_params_t *p, radar_det_t *dets,
                              uint32_t *rejected);
void tracker_init(tracker_t *tk);
void tracker_update(tracker_t *tk, const radar_det_t *d, int n, uint64_t t_ns, const aeb_params_t *p);
int tracker_export(const tracker_t *tk, radar_track_t *out, int max);
void radar_select_threat(const radar_track_t *tracks, int n, const aeb_params_t *p, threat_t *out);
#endif
