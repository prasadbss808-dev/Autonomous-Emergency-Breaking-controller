#ifndef VISION_SHM_H
#define VISION_SHM_H
#include <stdint.h>
#include <string.h>
#define VISION_SHM_NAME "/aeb_vision"
#define VISION_FRAME_NAME "/aeb_vision_frame"
#define VISION_SHM_MAGIC 0x56495331u
#define VISION_SHM_VERSION 2u
#define VISION_LABEL_LEN 16
#define VISION_MAX_DETS 10
#define VISION_JPEG_MAX (512u * 1024u)
typedef struct {
    int32_t class_id;
    float confidence;
    float x1, y1, x2, y2;
    float bearing_deg;
    uint8_t ahead;
    uint8_t pad[3];
    char label[VISION_LABEL_LEN];
} vision_det_t;
typedef struct {
    uint32_t seq;
    uint32_t magic;
    uint32_t version;
    uint32_t pid;
    uint64_t t_ns;
    uint64_t frame_no;
    float fps;
    float infer_ms;
    uint32_t n_det;
    int32_t class_id;
    float confidence;
    float bearing_deg;
    float area_frac;
    char label[VISION_LABEL_LEN];
    vision_det_t det[VISION_MAX_DETS];
    char source[24];
} vision_shm_t;
typedef struct {
    uint32_t seq;
    uint32_t magic;
    uint64_t t_ns;
    uint64_t frame_no;
    uint32_t width, height;
    uint32_t size;
    uint32_t pad;
    uint8_t data[VISION_JPEG_MAX];
} vision_frame_shm_t;
static inline void vision_seq_write_begin(uint32_t *seq)
{
    uint32_t q = __atomic_load_n(seq, __ATOMIC_RELAXED);
    __atomic_store_n(seq, q + 1u, __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_RELEASE);
}
static inline void vision_seq_write_end(uint32_t *seq)
{
    uint32_t q = __atomic_load_n(seq, __ATOMIC_RELAXED);
    __atomic_store_n(seq, q + 1u, __ATOMIC_RELEASE);
}
static inline int vision_seq_read(const void *obj, void *out, size_t n, int tries)
{
    const uint32_t *seq = (const uint32_t *)obj;
    while (tries-- > 0) {
        uint32_t q1 = __atomic_load_n(seq, __ATOMIC_ACQUIRE);
        if (q1 & 1u) continue;
        memcpy(out, obj, n);
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        uint32_t q2 = __atomic_load_n(seq, __ATOMIC_RELAXED);
        if (q1 == q2) return 1;
    }
    return 0;
}
static inline void vision_shm_write_begin(vision_shm_t *s)
{
    vision_seq_write_begin(&s->seq);
}
static inline void vision_shm_write_end(vision_shm_t *s)
{
    vision_seq_write_end(&s->seq);
}
static inline int vision_shm_read(const vision_shm_t *s, vision_shm_t *out, int tries)
{
    return vision_seq_read(s, out, sizeof *out, tries);
}
#endif
