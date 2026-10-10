#include "radar_track.h"
#include "aeb_logic.h"
#include <string.h>
static float fabs_(float v)
{
    return v < 0 ? -v : v;
}
int radar_frame_to_detections(const rd03d_frame_t *f, const aeb_params_t *p, radar_det_t *dets,
                              uint32_t *rejected)
{
    int n = 0;
    for (int i = 0; i < RD03D_NUM_SLOTS; i++) {
        const rd03d_slot_t *s = &f->slot[i];
        if (!s->present) continue;
        if (s->speed_sentinel && p->radar_drop_sentinel_speed) {
            (*rejected)++;
            continue;
        }
        float x = (float)s->x_mm / 1000.0f;
        float y = (float)s->y_mm / 1000.0f;
        float r = aeb_sqrtf(x * x + y * y);
        float v = (float)s->speed_cms / 100.0f;
        if (y <= 0.0f || r < p->radar_min_range_m || r > p->radar_max_range_m ||
            fabs_(x) > p->radar_max_lateral_m || fabs_(v) > p->radar_max_speed_mps) {
            (*rejected)++;
            continue;
        }
        radar_det_t *d = &dets[n++];
        d->slot = (uint8_t)i;
        d->x_m = x;
        d->y_m = y;
        d->range_m = r;
        d->v_radial_mps = v;
        d->doppler_valid = !s->speed_sentinel;
        d->res_m = (float)s->res_mm / 1000.0f;
    }
    return n;
}
void tracker_init(tracker_t *tk)
{
    memset(tk, 0, sizeof(*tk));
    tk->next_id = 1;
}
static void track_derive(radar_track_t *t, const aeb_params_t *p)
{
    if (t->doppler_valid)
        t->v_close_mps = -t->v_radial_mps;
    else if (t->rangerate_valid)
        t->v_close_mps = t->v_rangerate_close_mps;
    else
        t->v_close_mps = 0.0f;
    t->confirmed = t->hits >= (uint16_t)p->track_confirm_hits;
    t->in_corridor = (t->y_m > 0.0f) && (fabs_(t->x_m) <= p->corridor_half_width_m);
}
void tracker_update(tracker_t *tk, const radar_det_t *d, int n, uint64_t t_ns, const aeb_params_t *p)
{
    bool det_used[RD03D_NUM_SLOTS] = {false};
    bool trk_used[RADAR_MAX_TRACKS] = {false};
    const float gate2 = p->track_gate_m * p->track_gate_m;
    for (;;) {
        int bi = -1, bj = -1;
        float best = gate2;
        for (int i = 0; i < tk->n; i++) {
            if (trk_used[i]) continue;
            for (int j = 0; j < n; j++) {
                if (det_used[j]) continue;
                float dx = tk->t[i].pub.x_m - d[j].x_m;
                float dy = tk->t[i].pub.y_m - d[j].y_m;
                float d2 = dx * dx + dy * dy;
                if (d2 <= best) {
                    best = d2;
                    bi = i;
                    bj = j;
                }
            }
        }
        if (bi < 0) break;
        trk_used[bi] = det_used[bj] = true;
        trk_int_t *ti = &tk->t[bi];
        radar_track_t *t = &ti->pub;
        const radar_det_t *m = &d[bj];
        float dt = (t_ns > t->t_last_ns) ? (float)(t_ns - t->t_last_ns) * 1e-9f : 0.0f;
        const float ar = p->track_range_alpha, av = p->track_vel_alpha;
        t->x_m += ar * (m->x_m - t->x_m);
        t->y_m += ar * (m->y_m - t->y_m);
        t->range_m += ar * (m->range_m - t->range_m);
        if (m->doppler_valid) {
            t->v_radial_mps = t->doppler_valid ? t->v_radial_mps + av * (m->v_radial_mps - t->v_radial_mps)
                                               : m->v_radial_mps;
            t->doppler_valid = true;
        } else {
            t->doppler_valid = false;
        }
        if (dt > 0.001f && dt < 0.5f) {
            float rr = -(m->range_m - ti->last_raw_range_m) / dt;
            t->v_rangerate_close_mps =
                t->rangerate_valid ? t->v_rangerate_close_mps + av * (rr - t->v_rangerate_close_mps) : rr;
            t->rangerate_valid = true;
        }
        ti->last_raw_range_m = m->range_m;
        t->res_m = m->res_m;
        t->slot = m->slot;
        if (t->hits < 0xFFFF) t->hits++;
        t->misses = 0;
        t->t_last_ns = t_ns;
        track_derive(t, p);
    }
    for (int i = 0; i < tk->n;) {
        if (!trk_used[i]) {
            radar_track_t *t = &tk->t[i].pub;
            t->misses++;
            if (t->misses > (uint16_t)p->track_max_misses) {
                tk->t[i] = tk->t[tk->n - 1];
                trk_used[i] = trk_used[tk->n - 1];
                tk->n--;
                continue;
            }
        }
        i++;
    }
    for (int j = 0; j < n; j++) {
        if (det_used[j] || tk->n >= RADAR_MAX_TRACKS) continue;
        trk_int_t *ti = &tk->t[tk->n++];
        memset(ti, 0, sizeof(*ti));
        radar_track_t *t = &ti->pub;
        t->id = tk->next_id++;
        if (tk->next_id == 0) tk->next_id = 1;
        t->slot = d[j].slot;
        t->x_m = d[j].x_m;
        t->y_m = d[j].y_m;
        t->range_m = d[j].range_m;
        t->v_radial_mps = d[j].v_radial_mps;
        t->doppler_valid = d[j].doppler_valid;
        t->res_m = d[j].res_m;
        t->hits = 1;
        t->t_last_ns = t_ns;
        ti->last_raw_range_m = d[j].range_m;
        track_derive(t, p);
    }
}
int tracker_export(const tracker_t *tk, radar_track_t *out, int max)
{
    int n = tk->n < max ? tk->n : max;
    for (int i = 0; i < n; i++) out[i] = tk->t[i].pub;
    return n;
}
void radar_select_threat(const radar_track_t *tracks, int n, const aeb_params_t *p, threat_t *out)
{
    memset(out, 0, sizeof(*out));
    out->n_detected = (uint8_t)n;
    int best = -1;
    risk_level_t best_lvl = RISK_NONE;
    float best_ttc = 1e9f, best_r = 1e9f;
    for (int i = 0; i < n; i++) {
        const radar_track_t *t = &tracks[i];
        if (!t->confirmed || !t->in_corridor) continue;
        out->n_in_corridor++;
        threat_t cand = {true, t->id, t->range_m, t->x_m, t->v_close_mps, t->v_close_mps > p->v_close_min_mps,
                         0,    0};
        risk_eval_t ev;
        risk_level_t lvl = aeb_classify(&cand, p, 1.0f, &ev);
        bool better = false;
        if (best < 0 || lvl > best_lvl)
            better = true;
        else if (lvl == best_lvl) {
            if (ev.ttc_s < best_ttc - 1e-4f)
                better = true;
            else if (ev.ttc_s <= best_ttc + 1e-4f && t->range_m < best_r)
                better = true;
        }
        if (better) {
            best = i;
            best_lvl = lvl;
            best_ttc = ev.ttc_s;
            best_r = t->range_m;
        }
    }
    if (best >= 0) {
        const radar_track_t *t = &tracks[best];
        out->valid = true;
        out->id = t->id;
        out->range_m = t->range_m;
        out->x_m = t->x_m;
        out->v_close_mps = t->v_close_mps;
        out->closing = t->v_close_mps > p->v_close_min_mps;
    }
}
