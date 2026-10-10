#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#if !defined(AEB_NO_CAMAPI) && (defined(__QNX__) || defined(AEB_FORCE_CAMAPI))
#define AEB_HAVE_CAMAPI 1
#include <camera/camera_api.h>
#endif
#if defined(__QNX__)
#include <sys/neutrino.h>
#include <sys/syspage.h>
#endif
#include "vision_shm.h"
static const char *VOC_CLASSES[21] = {
    "background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
    "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
    "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};
struct Options {
    int unit = 3;
    std::string model_dir = "models";
    float conf_th = 0.50f;
    int req_w = 640;
    int req_h = 480;
    int max_w = 640;
    float hfov_deg = 66.0f;
    float corridor_deg = 20.0f;
    int threads = 2;
    unsigned runmask = 0xC;
    int prio = 8;
    std::vector<std::string> images;
    std::string out_path;
    float save_period_s = 2.0f;
    long max_frames = 0;
    bool quiet = false;
    bool publish = true;
    bool list_units = false;
    bool loop_images = false;
    bool verbose = false;
    bool no_winograd = false;
    bool diagnose = false;
    bool jpeg = true;
    bool no_announce = false;
    int replay_fps = 8;
};
static const float kPi = 3.14159265358979f;
static std::atomic<bool> g_stop{false};
static void on_signal(int)
{
    g_stop = true;
}
static uint64_t now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}
static void set_low_priority_and_runmask(const Options &o)
{
    struct sched_param sp;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = o.prio;
    int e = pthread_setschedparam(pthread_self(), SCHED_RR, &sp);
    if (e) fprintf(stderr, "VIS: warning: cannot set priority %d (%s)\n", o.prio, strerror(e));
#if defined(__QNX__)
    int ncpu = _syspage_ptr->num_cpu;
    int rsize = RMSK_SIZE(ncpu);
    size_t tot = sizeof(int) + 2u * (size_t)rsize * sizeof(unsigned);
    void *buf = calloc(1, tot);
    if (buf) {
        int *rsizep = (int *)buf;
        unsigned *rmaskp = (unsigned *)(rsizep + 1);
        unsigned *inheritp = rmaskp + rsize;
        *rsizep = rsize;
        int used = 0;
        for (int c = 0; c < ncpu && c < 32; c++) {
            if (o.runmask & (1u << c)) {
                RMSK_SET(c, rmaskp);
                RMSK_SET(c, inheritp);
                used++;
            }
        }
        if (used == 0) {
            for (int c = 0; c < ncpu; c++) {
                RMSK_SET(c, rmaskp);
                RMSK_SET(c, inheritp);
            }
        }
        if (ThreadCtl(_NTO_TCTL_RUNMASK_GET_AND_SET_INHERIT, buf) == -1) {
            unsigned m = o.runmask & ((ncpu >= 32) ? 0xFFFFFFFFu : ((1u << ncpu) - 1u));
            if (m && ThreadCtl(_NTO_TCTL_RUNMASK, (void *)(uintptr_t)m) != -1)
                fprintf(stderr,
                        "VIS: runmask 0x%X applied to the inference thread (inherit mask not supported)\n",
                        m);
            else
                fprintf(stderr, "VIS: note: runmask not applied (%s) - low priority still protects the AEB\n",
                        strerror(errno));
        }
        free(buf);
    }
#endif
}
static vision_shm_t *g_shm = nullptr;
static bool shm_open_writer()
{
    int fd = shm_open(VISION_SHM_NAME, O_RDWR | O_CREAT, 0666);
    if (fd < 0) {
        perror("VIS: shm_open " VISION_SHM_NAME);
        return false;
    }
    fchmod(fd, 0666);
    if (ftruncate(fd, sizeof(vision_shm_t)) != 0) {
        perror("VIS: ftruncate");
        close(fd);
        return false;
    }
    void *p = mmap(nullptr, sizeof(vision_shm_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED) {
        perror("VIS: mmap");
        return false;
    }
    g_shm = (vision_shm_t *)p;
    vision_shm_write_begin(g_shm);
    g_shm->magic = VISION_SHM_MAGIC;
    g_shm->version = VISION_SHM_VERSION;
    g_shm->pid = (uint32_t)getpid();
    g_shm->class_id = -1;
    g_shm->t_ns = 0;
    vision_shm_write_end(g_shm);
    return true;
}
struct Result {
    int n_det = 0;
    int class_id = -1;
    float conf = 0, bearing = 0, area = 0;
    float infer_ms = 0;
    std::vector<vision_det_t> dets;
};
static char g_source[24] = "camera";
static vision_frame_shm_t *g_frame = nullptr;
static bool frame_shm_open_writer()
{
    int fd = shm_open(VISION_FRAME_NAME, O_RDWR | O_CREAT, 0666);
    if (fd < 0) {
        perror("VIS: shm_open " VISION_FRAME_NAME);
        return false;
    }
    fchmod(fd, 0666);
    if (ftruncate(fd, sizeof(vision_frame_shm_t)) != 0) {
        perror("VIS: ftruncate frame");
        close(fd);
        return false;
    }
    void *p = mmap(nullptr, sizeof(vision_frame_shm_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED) {
        perror("VIS: mmap frame");
        return false;
    }
    g_frame = (vision_frame_shm_t *)p;
    vision_seq_write_begin(&g_frame->seq);
    g_frame->magic = VISION_SHM_MAGIC;
    g_frame->size = 0;
    vision_seq_write_end(&g_frame->seq);
    return true;
}
static void frame_publish(const cv::Mat &bgr, uint64_t frame_no)
{
    if (!g_frame || bgr.empty()) return;
    std::vector<uchar> jpg;
    std::vector<int> prm = {cv::IMWRITE_JPEG_QUALITY, 70};
    try {
        if (!cv::imencode(".jpg", bgr, jpg, prm)) return;
    } catch (const cv::Exception &) {
        return;
    }
    if (jpg.size() > VISION_JPEG_MAX) return;
    vision_seq_write_begin(&g_frame->seq);
    g_frame->t_ns = now_ns();
    g_frame->frame_no = frame_no;
    g_frame->width = (uint32_t)bgr.cols;
    g_frame->height = (uint32_t)bgr.rows;
    memcpy(g_frame->data, jpg.data(), jpg.size());
    g_frame->size = (uint32_t)jpg.size();
    vision_seq_write_end(&g_frame->seq);
}
static void announce(const Result &r)
{
    static uint64_t last_print[21];
    static bool present_prev[21];
    bool present[21] = {false};
    const vision_det_t *best[21] = {nullptr};
    for (const auto &d : r.dets) {
        if (d.class_id <= 0 || d.class_id >= 21) continue;
        present[d.class_id] = true;
        if (!best[d.class_id] || d.confidence > best[d.class_id]->confidence) best[d.class_id] = &d;
    }
    uint64_t now = now_ns();
    for (int c = 1; c < 21; c++) {
        if (present[c] && (!present_prev[c] || now - last_print[c] >= 2000000000ull)) {
            char up[VISION_LABEL_LEN];
            size_t i = 0;
            for (; VOC_CLASSES[c][i] && i < sizeof up - 1; i++)
                up[i] = (char)toupper((unsigned char)VOC_CLASSES[c][i]);
            up[i] = '\0';
            printf("OBJECT DETECTED: %-11s (conf %.2f, bearing %+.0f deg%s)\n", up, best[c]->confidence,
                   best[c]->bearing_deg, best[c]->ahead ? ", AHEAD" : "");
            last_print[c] = now;
        }
        present_prev[c] = present[c];
    }
    fflush(stdout);
}
static void shm_publish(const Result &r, uint64_t frame_no, float fps)
{
    if (!g_shm) return;
    vision_shm_write_begin(g_shm);
    g_shm->t_ns = now_ns();
    g_shm->frame_no = frame_no;
    g_shm->fps = fps;
    g_shm->infer_ms = r.infer_ms;
    g_shm->n_det = (uint32_t)r.n_det;
    g_shm->class_id = r.class_id;
    g_shm->confidence = r.conf;
    g_shm->bearing_deg = r.bearing;
    g_shm->area_frac = r.area;
    memset(g_shm->label, 0, sizeof g_shm->label);
    if (r.class_id > 0 && r.class_id < 21)
        strncpy(g_shm->label, VOC_CLASSES[r.class_id], sizeof g_shm->label - 1);
    size_t n = std::min(r.dets.size(), (size_t)VISION_MAX_DETS);
    memset(g_shm->det, 0, sizeof g_shm->det);
    for (size_t i = 0; i < n; i++) g_shm->det[i] = r.dets[i];
    g_shm->n_det = (uint32_t)n;
    memset(g_shm->source, 0, sizeof g_shm->source);
    snprintf(g_shm->source, sizeof g_shm->source, "%s", g_source);
    vision_shm_write_end(g_shm);
}
static void shm_mark_dead()
{
    if (!g_shm) return;
    vision_shm_write_begin(g_shm);
    g_shm->t_ns = 0;
    g_shm->class_id = -1;
    vision_shm_write_end(g_shm);
}
static cv::dnn::Net g_net;
static void set_winograd(bool on)
{
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    g_net.enableWinograd(on);
#else
    (void)on;
#endif
}
static void dump_raw(const cv::Mat &img, const cv::Mat &blob, const cv::Mat &out)
{
    cv::Scalar im = cv::mean(img);
    double bmin, bmax;
    cv::minMaxLoc(blob.reshape(1, 1), &bmin, &bmax);
    cv::Scalar bm = cv::mean(blob.reshape(1, 1));
    cv::Mat det(out.size[2], out.size[3], CV_32F, (void *)out.ptr<float>());
    int nan = 0;
    for (int i = 0; i < det.rows; i++)
        for (int j = 0; j < det.cols; j++)
            if (std::isnan(det.at<float>(i, j))) nan++;
    printf("    raw: image mean B%.0f G%.0f R%.0f | blob min %.3f max %.3f mean %.4f | out %dx%d, NaN %d\n",
           im[0], im[1], im[2], bmin, bmax, bm[0], det.rows, det.cols, nan);
    std::vector<int> idx(det.rows);
    for (int i = 0; i < det.rows; i++) idx[i] = i;
    std::sort(idx.begin(), idx.end(),
              [&](int a, int b) { return det.at<float>(a, 2) > det.at<float>(b, 2); });
    for (int k = 0; k < std::min(5, det.rows); k++) {
        int i = idx[k];
        int cls = (int)det.at<float>(i, 1);
        printf("    top%d: class %2d %-11s conf %.3f box %.2f %.2f %.2f %.2f\n", k + 1, cls,
               (cls >= 0 && cls < 21) ? VOC_CLASSES[cls] : "?", det.at<float>(i, 2), det.at<float>(i, 3),
               det.at<float>(i, 4), det.at<float>(i, 5), det.at<float>(i, 6));
    }
}
static bool pb_varint(const uint8_t *&p, const uint8_t *end, uint64_t &v)
{
    v = 0;
    for (int sh = 0; sh < 64 && p < end; sh += 7) {
        uint8_t b = *p++;
        v |= (uint64_t)(b & 0x7f) << sh;
        if (!(b & 0x80)) return true;
    }
    return false;
}
static void swap_bytes(uint8_t *q, int n)
{
    std::reverse(q, q + n);
}
enum PbMsg { PB_NET, PB_LAYER, PB_V1LAYER, PB_BLOB };
static bool pb_walk(uint8_t *p0, uint8_t *end0, PbMsg m, size_t &nswapped)
{
    const uint8_t *p = p0, *end = end0;
    while (p < end) {
        uint64_t tag;
        if (!pb_varint(p, end, tag)) return false;
        uint32_t field = (uint32_t)(tag >> 3), wt = (uint32_t)(tag & 7);
        bool blob_float = (m == PB_BLOB) && (field == 5 || field == 6);
        bool blob_double = (m == PB_BLOB) && (field == 8 || field == 9);
        switch (wt) {
        case 0: {
            uint64_t v;
            if (!pb_varint(p, end, v)) return false;
            break;
        }
        case 1:
            if (end - p < 8) return false;
            if (blob_double) {
                swap_bytes((uint8_t *)p, 8);
                nswapped++;
            }
            p += 8;
            break;
        case 5:
            if (end - p < 4) return false;
            if (blob_float) {
                swap_bytes((uint8_t *)p, 4);
                nswapped++;
            }
            p += 4;
            break;
        case 2: {
            uint64_t len;
            if (!pb_varint(p, end, len) || len > (uint64_t)(end - p)) return false;
            uint8_t *q = (uint8_t *)p, *qe = q + len;
            if (blob_float) {
                for (; q + 4 <= qe; q += 4) {
                    swap_bytes(q, 4);
                    nswapped++;
                }
            } else if (blob_double) {
                for (; q + 8 <= qe; q += 8) {
                    swap_bytes(q, 8);
                    nswapped++;
                }
            } else if (m == PB_NET && field == 100) {
                if (!pb_walk(q, qe, PB_LAYER, nswapped)) return false;
            } else if (m == PB_NET && field == 2) {
                if (!pb_walk(q, qe, PB_V1LAYER, nswapped)) return false;
            } else if ((m == PB_LAYER && field == 7) || (m == PB_V1LAYER && field == 6)) {
                if (!pb_walk(q, qe, PB_BLOB, nswapped)) return false;
            }
            p += len;
            break;
        }
        default: return false;
        }
    }
    return true;
}
static bool weights_sane(cv::dnn::Net &n, std::string &which)
{
    std::vector<cv::String> names = n.getLayerNames();
    for (const auto &ln : names) {
        cv::Mat w;
        try {
            w = n.getParam(ln, 0);
        } catch (const cv::Exception &) {
            continue;
        }
        if (w.empty() || w.depth() != CV_32F) continue;
        which = ln;
        const float *f = w.ptr<float>();
        for (size_t i = 0; i < w.total(); i++)
            if (!std::isfinite(f[i]) || std::fabs(f[i]) > 1e4f) return false;
        return true;
    }
    which = "(none)";
    return true;
}
static bool read_file(const std::string &path, std::vector<char> &buf)
{
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (n <= 0) {
        fclose(f);
        return false;
    }
    buf.resize((size_t)n);
    bool ok = fread(buf.data(), 1, (size_t)n, f) == (size_t)n;
    fclose(f);
    return ok;
}
static bool g_force_swap = false;
static bool load_model(const Options &o)
{
    std::string proto = o.model_dir + "/deploy.prototxt";
    std::string model = o.model_dir + "/mobilenet_iter_73000.caffemodel";
    struct stat st;
    if (stat(model.c_str(), &st) == 0 && st.st_size != 23306119)
        fprintf(stderr, "VIS: WARNING: %s is %lld bytes, expected 23306119 (corrupt/incomplete copy?)\n",
                model.c_str(), (long long)st.st_size);
    try {
        g_net = cv::dnn::readNetFromCaffe(proto, model);
    } catch (const cv::Exception &e) {
        fprintf(stderr, "VIS: cannot load model (%s, %s): %s\n", proto.c_str(), model.c_str(), e.what());
        return false;
    }
    if (g_net.empty()) {
        fprintf(stderr, "VIS: model is empty\n");
        return false;
    }
    std::string which;
    if (g_force_swap || !weights_sane(g_net, which)) {
        if (!g_force_swap)
            fprintf(stderr,
                    "VIS: weights of '%s' are invalid after loading -> this OpenCV build reads floats "
                    "byte-swapped; applying the byte-swap work-around\n",
                    which.c_str());
        std::vector<char> pbuf, mbuf;
        if (!read_file(proto, pbuf) || !read_file(model, mbuf)) {
            fprintf(stderr, "VIS: cannot read model files\n");
            return false;
        }
        size_t nsw = 0;
        if (!pb_walk((uint8_t *)mbuf.data(), (uint8_t *)mbuf.data() + mbuf.size(), PB_NET, nsw)) {
            fprintf(stderr, "VIS: caffemodel parse error in work-around\n");
            return false;
        }
        try {
            g_net = cv::dnn::readNetFromCaffe(pbuf.data(), pbuf.size(), mbuf.data(), mbuf.size());
        } catch (const cv::Exception &e) {
            fprintf(stderr, "VIS: reload failed: %s\n", e.what());
            return false;
        }
        bool ok = weights_sane(g_net, which);
        fprintf(stderr, "VIS: byte-swapped %zu weight values -> weights %s\n", nsw,
                ok ? "OK" : "STILL INVALID");
        if (!ok) return false;
    }
    g_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    g_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    set_winograd(!o.no_winograd);
    return true;
}
static Result detect(const Options &o, cv::Mat &bgr, bool draw)
{
    Result r;
    auto t0 = std::chrono::steady_clock::now();
    cv::Mat blob = cv::dnn::blobFromImage(bgr, 0.007843, cv::Size(300, 300), cv::Scalar(127.5, 127.5, 127.5),
                                          false, false);
    g_net.setInput(blob);
    cv::Mat out = g_net.forward();
    r.infer_ms = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (o.verbose) dump_raw(bgr, blob, out);
    cv::Mat det(out.size[2], out.size[3], CV_32F, out.ptr<float>());
    const float tan_half = std::tan(o.hfov_deg * 0.5f * kPi / 180.0f);
    float best_area = -1.0f;
    for (int i = 0; i < det.rows; i++) {
        float conf = det.at<float>(i, 2);
        int cls = (int)det.at<float>(i, 1);
        if (conf < o.conf_th || cls <= 0 || cls >= 21) continue;
        float x1 = std::min(std::max(det.at<float>(i, 3), 0.f), 1.f);
        float y1 = std::min(std::max(det.at<float>(i, 4), 0.f), 1.f);
        float x2 = std::min(std::max(det.at<float>(i, 5), 0.f), 1.f);
        float y2 = std::min(std::max(det.at<float>(i, 6), 0.f), 1.f);
        if (x2 <= x1 || y2 <= y1) continue;
        r.n_det++;
        float cx = 0.5f * (x1 + x2);
        float bearing = std::atan((2.0f * cx - 1.0f) * tan_half) * 180.0f / kPi;
        float area = (x2 - x1) * (y2 - y1);
        bool ahead = std::fabs(bearing) <= o.corridor_deg;
        if (r.dets.size() < VISION_MAX_DETS) {
            vision_det_t d;
            memset(&d, 0, sizeof d);
            d.class_id = cls;
            d.confidence = conf;
            d.x1 = x1;
            d.y1 = y1;
            d.x2 = x2;
            d.y2 = y2;
            d.bearing_deg = bearing;
            d.ahead = ahead ? 1 : 0;
            strncpy(d.label, VOC_CLASSES[cls], sizeof d.label - 1);
            r.dets.push_back(d);
        }
        if (ahead && area > best_area) {
            best_area = area;
            r.class_id = cls;
            r.conf = conf;
            r.bearing = bearing;
            r.area = area;
        }
        if (draw) {
            cv::Rect box((int)(x1 * bgr.cols), (int)(y1 * bgr.rows), (int)((x2 - x1) * bgr.cols),
                         (int)((y2 - y1) * bgr.rows));
            cv::Scalar col = ahead ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 200, 0);
            cv::rectangle(bgr, box, col, 2);
            char txt[64];
            snprintf(txt, sizeof txt, "%s %.2f %+.0fdeg", VOC_CLASSES[cls], conf, bearing);
            cv::putText(bgr, txt, cv::Point(box.x, std::max(box.y - 4, 12)), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        col, 1);
        }
    }
    if (draw) {
        float xl = 0.5f * (1.0f + std::tan(-o.corridor_deg * kPi / 180.f) / tan_half);
        float xr = 0.5f * (1.0f + std::tan(o.corridor_deg * kPi / 180.f) / tan_half);
        cv::line(bgr, cv::Point((int)(xl * bgr.cols), 0), cv::Point((int)(xl * bgr.cols), bgr.rows),
                 cv::Scalar(255, 255, 0), 1);
        cv::line(bgr, cv::Point((int)(xr * bgr.cols), 0), cv::Point((int)(xr * bgr.cols), bgr.rows),
                 cv::Scalar(255, 255, 0), 1);
    }
    return r;
}
static void print_result(const Result &r, uint64_t frame_no, float fps)
{
    if (r.class_id > 0)
        printf(
            "VIS frame %6llu | %4.1f fps | infer %5.1f ms | %u det | AHEAD: %-10s conf %.2f bearing %+5.1f deg size %.3f\n",
            (unsigned long long)frame_no, fps, r.infer_ms, (unsigned)r.n_det, VOC_CLASSES[r.class_id], r.conf,
            r.bearing, r.area);
    else
        printf("VIS frame %6llu | %4.1f fps | infer %5.1f ms | %u det | AHEAD: none\n",
               (unsigned long long)frame_no, fps, r.infer_ms, (unsigned)r.n_det);
    fflush(stdout);
}
static int run_images_loop(const Options &o)
{
    std::vector<cv::Mat> imgs;
    for (const auto &f : o.images) {
        cv::Mat m = cv::imread(f);
        if (m.empty()) {
            fprintf(stderr, "VIS: cannot read %s\n", f.c_str());
            return 1;
        }
        imgs.push_back(m);
    }
    uint64_t frame_no = 0, t_last = now_ns(), n_last = 0;
    float fps = 0;
    while (!g_stop) {
        cv::Mat img = imgs[(frame_no / 30) % imgs.size()].clone();
        Result r = detect(o, img, o.jpeg);
        frame_no++;
        shm_publish(r, frame_no, fps);
        if (o.jpeg) frame_publish(img, frame_no);
        if (!o.no_announce) announce(r);
        if (o.replay_fps > 0) usleep((useconds_t)(1000000 / o.replay_fps));
        uint64_t t = now_ns();
        if (t - t_last >= 1000000000ull) {
            fps = (float)(frame_no - n_last) * 1e9f / (float)(t - t_last);
            n_last = frame_no;
            t_last = t;
            if (!o.quiet) print_result(r, frame_no, fps);
        }
        if (o.max_frames > 0 && (long)frame_no >= o.max_frames) break;
    }
    return 0;
}
static void layer_stats(const char *name, const cv::Mat &m)
{
    const float *p = m.ptr<float>();
    size_t n = m.total();
    double mn = 1e30, mx = -1e30, sum = 0;
    size_t nan = 0, inf = 0, zero = 0;
    for (size_t i = 0; i < n; i++) {
        float v = p[i];
        if (std::isnan(v)) {
            nan++;
            continue;
        }
        if (std::isinf(v)) {
            inf++;
            continue;
        }
        if (v == 0.0f) zero++;
        mn = std::min(mn, (double)v);
        mx = std::max(mx, (double)v);
        sum += v;
    }
    printf("  %-20s n=%-8zu min %11.4f max %11.4f mean %10.5f  zero %5.1f%%  NaN %zu Inf %zu\n", name, n, mn,
           mx, n ? sum / (double)n : 0.0, n ? 100.0 * (double)zero / (double)n : 0.0, nan, inf);
}
static void math_selftest()
{
    cv::Mat a(1, 8, CV_32F), e;
    for (int i = 0; i < 8; i++) a.at<float>(i) = -4.0f + (float)i;
    cv::exp(a, e);
    double maxerr = 0;
    for (int i = 0; i < 8; i++)
        maxerr = std::max(maxerr, std::fabs((double)e.at<float>(i) - std::exp((double)a.at<float>(i))) /
                                      std::exp((double)a.at<float>(i)));
    printf("  cv::exp            rel.err %.2e  (%s)\n", maxerr, maxerr < 1e-5 ? "OK" : "WRONG");
    cv::Mat A(64, 64, CV_32F), B(64, 64, CV_32F), C;
    cv::randu(A, -1, 1);
    cv::randu(B, -1, 1);
    cv::gemm(A, B, 1.0, cv::noArray(), 0.0, C);
    double err = 0;
    for (int i = 0; i < 64; i++)
        for (int j = 0; j < 64; j++) {
            double r = 0;
            for (int k = 0; k < 64; k++) r += (double)A.at<float>(i, k) * B.at<float>(k, j);
            err = std::max(err, std::fabs(r - C.at<float>(i, j)));
        }
    printf("  cv::gemm 64x64     max.err %.2e  (%s)\n", err, err < 1e-3 ? "OK" : "WRONG");
}
static double conv_ref_maxerr(int cin, int cout, int k, int group, int stride, int threads)
{
    cv::setNumThreads(threads);
    const int H = 19, W = 19, pad = k / 2, cin_g = cin / group, cout_g = cout / group;
    cv::dnn::LayerParams lp;
    lp.name = "c";
    lp.type = "Convolution";
    lp.set("kernel_size", k);
    lp.set("num_output", cout);
    lp.set("pad", pad);
    lp.set("stride", stride);
    lp.set("bias_term", false);
    lp.set("group", group);
    int wsz[4] = {cout, cin_g, k, k};
    cv::Mat w(4, wsz, CV_32F);
    cv::randu(w, -1, 1);
    lp.blobs.push_back(w);
    cv::dnn::Net n;
    n.addLayerToPrev(lp.name, lp.type, lp);
    int isz[4] = {1, cin, H, W};
    cv::Mat in(4, isz, CV_32F);
    cv::randu(in, -1, 1);
    n.setInput(in);
    cv::Mat o = n.forward();
    const float *I = in.ptr<float>(), *Wt = w.ptr<float>(), *O = o.ptr<float>();
    const int Ho = (H + 2 * pad - k) / stride + 1, Wo = (W + 2 * pad - k) / stride + 1;
    if (o.dims != 4 || o.size[2] != Ho || o.size[3] != Wo) return 1e30;
    double err = 0;
    for (int oc = 0; oc < cout; oc++) {
        int g = oc / cout_g;
        for (int y = 0; y < Ho; y++)
            for (int x = 0; x < Wo; x++) {
                double acc = 0;
                for (int ic = 0; ic < cin_g; ic++)
                    for (int ky = 0; ky < k; ky++)
                        for (int kx = 0; kx < k; kx++) {
                            int yy = y * stride + ky - pad, xx = x * stride + kx - pad;
                            if (yy < 0 || yy >= H || xx < 0 || xx >= W) continue;
                            acc += (double)I[((g * cin_g + ic) * H + yy) * W + xx] *
                                   Wt[((oc * cin_g + ic) * k + ky) * k + kx];
                        }
                double v = O[(oc * Ho + y) * Wo + x];
                double e = std::isfinite(v) ? std::fabs(v - acc) : 1e30;
                err = std::max(err, e);
            }
    }
    return err;
}
static void weight_stats(const Options &o)
{
    (void)o;
    cv::dnn::Net &n = g_net;
    const char *layers[] = {"conv0", "conv0/bn", "conv0/scale", "conv1/dw", "conv13", "conv17_2_mbox_conf"};
    for (const char *ln : layers) {
        for (int i = 0; i < 4; i++) {
            cv::Mat p;
            try {
                p = n.getParam(cv::String(ln), i);
            } catch (const cv::Exception &) {
                break;
            }
            if (p.empty()) break;
            double mn, mx;
            cv::minMaxLoc(p.reshape(1, 1), &mn, &mx);
            printf("  %-20s blob%d n=%-8zu min %10.5f max %10.5f mean %11.6f sum|x| %12.4f\n", ln, i,
                   p.total(), mn, mx, cv::mean(p.reshape(1, 1))[0], cv::norm(p.reshape(1, 1), cv::NORM_L1));
        }
    }
}
static int run_diagnose(const Options &o0)
{
    cv::Mat img = cv::imread(o0.images[0]);
    if (img.empty()) {
        fprintf(stderr, "VIS: cannot read %s\n", o0.images[0].c_str());
        return 1;
    }
    printf("DIAG image %s %dx%d, OpenCV %s\n", o0.images[0].c_str(), img.cols, img.rows, CV_VERSION);
    const int thr[2] = {o0.threads, 1};
    const bool wino[2] = {true, false};
    for (int t = 0; t < 2; t++)
        for (int w = 0; w < 2; w++) {
            Options o = o0;
            o.verbose = true;
            cv::setNumThreads(thr[t]);
            set_winograd(wino[w]);
            cv::Mat m = img.clone();
            detect(o, m, false);
            printf("DIAG threads=%d winograd=%s\n", thr[t], wino[w] ? "on" : "off");
            Result r = detect(o, m, false);
            print_result(r, 1, 0.0f);
        }
    printf("DIAG math self-test\n");
    math_selftest();
    printf("DIAG model weights as loaded (compare with the PC)\n");
    weight_stats(o0);
    printf("DIAG convolution unit tests (max error, must be < 1e-3)\n");
    {
        struct {
            const char *name;
            int cin, cout, k, group, stride;
        } t[] = {{"3x3 full   3->32  s1", 3, 32, 3, 1, 1},   {"3x3 full   3->32  s2", 3, 32, 3, 1, 2},
                 {"3x3 depthw 32->32 s1", 32, 32, 3, 32, 1}, {"3x3 depthw 32->32 s2", 32, 32, 3, 32, 2},
                 {"1x1 point  32->64   ", 32, 64, 1, 1, 1},  {"1x1 point 64->128   ", 64, 128, 1, 1, 1}};
        for (auto &c : t)
            for (int th = 1; th <= 2; th++) {
                double e = -1;
                try {
                    e = conv_ref_maxerr(c.cin, c.cout, c.k, c.group, c.stride, th);
                } catch (const cv::Exception &ex) {
                    printf("  %s threads=%d  EXCEPTION %s\n", c.name, th, ex.what());
                    continue;
                }
                printf("  %s threads=%d  max.err %.3g  (%s)\n", c.name, th, e, e < 1e-3 ? "OK" : "WRONG");
            }
    }
    printf("DIAG layer outputs (compare with the reference values from the PC)\n");
    {
        cv::setNumThreads(1);
        set_winograd(false);
        std::vector<cv::String> names = {
            "conv0",        "conv1",    "conv5",     "conv11",        "conv13",
            "conv17_2",     "mbox_loc", "mbox_conf", "mbox_priorbox", "mbox_conf_softmax",
            "detection_out"};
        cv::Mat blob = cv::dnn::blobFromImage(img, 0.007843, cv::Size(300, 300),
                                              cv::Scalar(127.5, 127.5, 127.5), false, false);
        try {
            g_net.setInput(blob);
            std::vector<cv::Mat> outs;
            g_net.forward(outs, names);
            for (size_t i = 0; i < outs.size() && i < names.size(); i++)
                layer_stats(names[i].c_str(), outs[i]);
        } catch (const cv::Exception &e) {
            printf("  layer dump failed: %s\n", e.what());
        }
    }
    return 0;
}
static int run_images(const Options &o)
{
    if (o.diagnose) return run_diagnose(o);
    if (o.loop_images) return run_images_loop(o);
    int rc = 0;
    for (size_t k = 0; k < o.images.size(); k++) {
        cv::Mat img = cv::imread(o.images[k]);
        if (img.empty()) {
            fprintf(stderr, "VIS: cannot read %s\n", o.images[k].c_str());
            rc = 1;
            continue;
        }
        detect(o, img, false);
        Result r = detect(o, img, !o.out_path.empty() || o.jpeg);
        printf("IMG %s (%dx%d): ", o.images[k].c_str(), img.cols, img.rows);
        print_result(r, k + 1, 0.0f);
        if (!o.no_announce) announce(r);
        if (o.jpeg) frame_publish(img, k + 1);
        if (g_shm) shm_publish(r, k + 1, 0.0f);
        if (!o.out_path.empty()) {
            std::string out = o.out_path;
            if (o.images.size() > 1) {
                size_t dot = out.rfind('.');
                std::string idx = "_" + std::to_string(k + 1);
                out = (dot == std::string::npos) ? out + idx : out.substr(0, dot) + idx + out.substr(dot);
            }
            if (cv::imwrite(out, img))
                printf("    annotated image -> %s\n", out.c_str());
            else
                fprintf(stderr, "VIS: cannot write %s\n", out.c_str());
        }
    }
    return rc;
}
#ifdef AEB_HAVE_CAMAPI
struct CamCtx {
    camera_handle_t handle = (camera_handle_t)-1;
    int max_w = 640;
    std::mutex m;
    std::condition_variable cv;
    bool want = true;
    bool ready = false;
    cv::Mat frame;
    std::atomic<uint64_t> cb_frames{0}, cb_skipped{0}, cb_badtype{0};
    std::atomic<int> last_type{-1};
    std::atomic<int> src_w{0}, src_h{0};
};
static void cam_status_cb(camera_handle_t, camera_devstatus_t status, uint16_t extra, void *)
{
    switch (status) {
    case CAMERA_STATUS_VIDEOVF: fprintf(stderr, "VIS: camera viewfinder streaming\n"); break;
    case CAMERA_STATUS_VIEWFINDER_ACTIVE: fprintf(stderr, "VIS: camera viewfinder active\n"); break;
    default: fprintf(stderr, "VIS: camera status %d (extra %u)\n", (int)status, (unsigned)extra); break;
    }
}
static void cam_vf_cb(camera_handle_t, camera_buffer_t *b, void *arg)
{
    CamCtx *c = (CamCtx *)arg;
    c->cb_frames++;
    c->last_type = (int)b->frametype;
    {
        std::lock_guard<std::mutex> lk(c->m);
        if (!c->want) {
            c->cb_skipped++;
            return;
        }
    }
    if (b->frametype != CAMERA_FRAMETYPE_NV12) {
        cv::Mat bgr;
        switch (b->frametype) {
        case CAMERA_FRAMETYPE_YCBYCR: {
            cv::Mat raw(b->framedesc.ycbycr.height, b->framedesc.ycbycr.width, CV_8UC2, b->framebuf,
                        b->framedesc.ycbycr.stride);
            cv::cvtColor(raw, bgr, cv::COLOR_YUV2BGR_YUY2);
            break;
        }
        case CAMERA_FRAMETYPE_CBYCRY: {
            cv::Mat raw(b->framedesc.cbycry.height, b->framedesc.cbycry.width, CV_8UC2, b->framebuf,
                        b->framedesc.cbycry.stride);
            cv::cvtColor(raw, bgr, cv::COLOR_YUV2BGR_UYVY);
            break;
        }
        case CAMERA_FRAMETYPE_RGB888: {
            cv::Mat raw(b->framedesc.rgb888.height, b->framedesc.rgb888.width, CV_8UC3, b->framebuf,
                        b->framedesc.rgb888.stride);
            cv::cvtColor(raw, bgr, cv::COLOR_RGB2BGR);
            break;
        }
        case CAMERA_FRAMETYPE_RGB8888: {
            cv::Mat raw(b->framedesc.rgb8888.height, b->framedesc.rgb8888.width, CV_8UC4, b->framebuf,
                        b->framedesc.rgb8888.stride);
            cv::cvtColor(raw, bgr, cv::COLOR_BGRA2BGR);
            break;
        }
        case CAMERA_FRAMETYPE_BGR8888: {
            cv::Mat raw(b->framedesc.bgr8888.height, b->framedesc.bgr8888.width, CV_8UC4, b->framebuf,
                        b->framedesc.bgr8888.stride);
            cv::cvtColor(raw, bgr, cv::COLOR_BGRA2BGR);
            break;
        }
        default: c->cb_badtype++; return;
        }
        c->src_w = bgr.cols;
        c->src_h = bgr.rows;
        if (bgr.cols > c->max_w) {
            int dw = c->max_w & ~1;
            int dh = ((int)((float)bgr.rows * (float)dw / (float)bgr.cols)) & ~1;
            cv::Mat small;
            cv::resize(bgr, small, cv::Size(dw, dh), 0, 0, cv::INTER_AREA);
            bgr = small;
        }
        {
            std::lock_guard<std::mutex> lk(c->m);
            c->frame = bgr;
            c->ready = true;
            c->want = false;
        }
        c->cv.notify_one();
        return;
    }
    const int w = (int)b->framedesc.nv12.width;
    const int h = (int)b->framedesc.nv12.height;
    c->src_w = w;
    c->src_h = h;
    cv::Mat y(h, w, CV_8UC1, b->framebuf, b->framedesc.nv12.stride);
    cv::Mat uv(h / 2, w / 2, CV_8UC2, b->framebuf + b->framedesc.nv12.uv_offset, b->framedesc.nv12.uv_stride);
    cv::Mat bgr;
    if (w > c->max_w) {
        int dw = c->max_w & ~1;
        int dh = ((int)((float)h * (float)dw / (float)w)) & ~1;
        cv::Mat ys, uvs;
        cv::resize(y, ys, cv::Size(dw, dh), 0, 0, cv::INTER_AREA);
        cv::resize(uv, uvs, cv::Size(dw / 2, dh / 2), 0, 0, cv::INTER_AREA);
        cv::cvtColorTwoPlane(ys, uvs, bgr, cv::COLOR_YUV2BGR_NV12);
    } else {
        cv::cvtColorTwoPlane(y, uv, bgr, cv::COLOR_YUV2BGR_NV12);
    }
    {
        std::lock_guard<std::mutex> lk(c->m);
        c->frame = bgr;
        c->ready = true;
        c->want = false;
    }
    c->cv.notify_one();
}
static void list_units()
{
    unsigned int n = 0;
    if (camera_get_supported_cameras(0, &n, nullptr) != CAMERA_EOK) {
        fprintf(stderr, "VIS: camera_get_supported_cameras failed\n");
        return;
    }
    std::vector<camera_unit_t> u(n);
    if (n && camera_get_supported_cameras(n, &n, u.data()) != CAMERA_EOK) {
        fprintf(stderr, "VIS: query failed\n");
        return;
    }
    printf("VIS: %u camera unit(s):", n);
    for (unsigned i = 0; i < n; i++) printf(" %d", (int)u[i]);
    printf(
        "\n(QNX 8.0.5 QSTI sensor_rpi5.conf: 1=simulator, 2=file playback, 3=Camera Module 3 on DISP0, 4=DISP1)\n");
}
static int run_camera(const Options &o)
{
    CamCtx c;
    c.max_w = o.max_w;
    int e =
        camera_open((camera_unit_t)o.unit, CAMERA_MODE_RO | CAMERA_MODE_ROLL | CAMERA_MODE_PWRITE, &c.handle);
    if (e != CAMERA_EOK) {
        fprintf(stderr,
                "VIS: camera_open(unit %d) failed: %d (%s)\n"
                "     check: ls -l /dev/sensor ; pidin ar | grep sensor ; try -L to list units\n",
                o.unit, e, strerror(e));
        return 2;
    }
    fprintf(stderr, "VIS: frame type ids: NV12=%d YCBYCR=%d CBYCRY=%d RGB888=%d RGB8888=%d BGR8888=%d\n",
            (int)CAMERA_FRAMETYPE_NV12, (int)CAMERA_FRAMETYPE_YCBYCR, (int)CAMERA_FRAMETYPE_CBYCRY,
            (int)CAMERA_FRAMETYPE_RGB888, (int)CAMERA_FRAMETYPE_RGB8888, (int)CAMERA_FRAMETYPE_BGR8888);
    camera_frametype_t ft = CAMERA_FRAMETYPE_UNSPECIFIED;
    if (camera_get_vf_property(c.handle, CAMERA_IMGPROP_FORMAT, &ft) == CAMERA_EOK)
        fprintf(stderr, "VIS: unit %d viewfinder format = %d\n", o.unit, (int)ft);
    if (ft != CAMERA_FRAMETYPE_NV12) {
        int se = camera_set_vf_property(c.handle, CAMERA_IMGPROP_FORMAT, CAMERA_FRAMETYPE_NV12);
        camera_frametype_t ft2 = ft;
        camera_get_vf_property(c.handle, CAMERA_IMGPROP_FORMAT, &ft2);
        fprintf(stderr, "VIS: request NV12 -> %s, format now %d\n", se == CAMERA_EOK ? "accepted" : "refused",
                (int)ft2);
    }
    if (camera_set_vf_property(c.handle, CAMERA_IMGPROP_CREATEWINDOW, false) != CAMERA_EOK)
        fprintf(stderr, "VIS: warning: could not disable viewfinder window\n");
    camera_set_vf_property(c.handle, CAMERA_IMGPROP_FRAMERATE, (double)30.0);
    camera_set_vf_property(c.handle, CAMERA_IMGPROP_WIDTH, o.req_w);
    camera_set_vf_property(c.handle, CAMERA_IMGPROP_HEIGHT, o.req_h);
    e = camera_start_viewfinder(c.handle, cam_vf_cb, cam_status_cb, &c);
    if (e != CAMERA_EOK) {
        fprintf(stderr, "VIS: camera_start_viewfinder failed: %d (%s)\n", e, strerror(e));
        camera_close(c.handle);
        return 2;
    }
    fprintf(stderr, "VIS: camera unit %d started; detector threshold %.2f, corridor +-%.0f deg\n", o.unit,
            o.conf_th, o.corridor_deg);
    uint64_t frame_no = 0, t_last_print = now_ns(), t_last_save = 0, frames_at_print = 0;
    float fps = 0.0f;
    int no_frame_s = 0;
    while (!g_stop) {
        cv::Mat bgr;
        {
            std::unique_lock<std::mutex> lk(c.m);
            c.want = true;
            if (!c.cv.wait_for(lk, std::chrono::seconds(1), [&] { return c.ready || g_stop.load(); })) {
                lk.unlock();
                no_frame_s++;
                fprintf(
                    stderr,
                    "VIS: no usable frame for %d s (callbacks %llu, last type %d, unsupported-type %llu)\n",
                    no_frame_s, (unsigned long long)c.cb_frames.load(), c.last_type.load(),
                    (unsigned long long)c.cb_badtype.load());
                continue;
            }
            if (g_stop) break;
            bgr = c.frame;
            c.frame.release();
            c.ready = false;
        }
        no_frame_s = 0;
        frame_no++;
        if (frame_no == 1)
            fprintf(stderr, "VIS: first frame: type %d, source %dx%d -> processed %dx%d BGR\n",
                    c.last_type.load(), c.src_w.load(), c.src_h.load(), bgr.cols, bgr.rows);
        uint64_t t = now_ns();
        bool save = !o.out_path.empty() && o.save_period_s > 0 &&
                    (t - t_last_save) >= (uint64_t)(o.save_period_s * 1e9f);
        Result r = detect(o, bgr, save || o.jpeg);
        shm_publish(r, frame_no, fps);
        if (o.jpeg) frame_publish(bgr, frame_no);
        if (!o.no_announce) announce(r);
        if (save) {
            t_last_save = t;
            cv::imwrite(o.out_path, bgr);
        }
        if (t - t_last_print >= 1000000000ull) {
            fps = (float)(frame_no - frames_at_print) * 1e9f / (float)(t - t_last_print);
            frames_at_print = frame_no;
            t_last_print = t;
            if (!o.quiet) {
                print_result(r, frame_no, fps);
            }
        }
        if (o.max_frames > 0 && (long)frame_no >= o.max_frames) break;
    }
    camera_stop_viewfinder(c.handle);
    camera_close(c.handle);
    fprintf(stderr, "VIS: camera closed (%llu frames processed, %llu callbacks, %llu skipped while busy)\n",
            (unsigned long long)frame_no, (unsigned long long)c.cb_frames.load(),
            (unsigned long long)c.cb_skipped.load());
    return 0;
}
#endif
static void usage(const char *p)
{
    printf("usage: %s [options]\n"
           "  -u N     camera unit (default 3 = Camera Module 3 on DISP0, QNX 8.0.5 QSTI)\n"
           "  -L       list camera units and exit\n"
           "  -m DIR   model dir with deploy.prototxt + mobilenet_iter_73000.caffemodel (default ./models)\n"
           "  -t TH    confidence threshold (default 0.50)\n"
           "  -W/-H N  requested viewfinder size hint (default 640x480)\n"
           "  -X N     downscale frames wider than N before detection (default 640)\n"
           "  -F DEG   camera horizontal FOV (default 66 = Camera Module 3; 102 for the wide version)\n"
           "  -k DEG   corridor half-angle counted as 'ahead' (default 20)\n"
           "  -j N     OpenCV threads (default 2)\n"
           "  -R HEX   CPU runmask (default 0xC = CPUs 2,3)\n"
           "  -P N     priority (default 8, below all AEB threads)\n"
           "  -o FILE  write annotated image (camera mode: every -s seconds)\n"
           "  -s SEC   save period for -o in camera mode (default 2)\n"
           "  -i FILE  offline test on an image instead of the camera (repeatable)\n"
           "  -r       with -i: replay the images forever and publish (camera stand-in)\n"
           "  -n N     stop after N frames\n"
           "  -x       do not publish to shared memory\n"
           "  -v       print raw network output (debug)\n"
           "  -w       disable Winograd convolution\n"
           "  -d       with -i: diagnose (threads 2/1 x Winograd on/off)\n"
           "  -S       force the float byte-swap work-around (test only)\n"
           "  -J       do not publish the annotated JPEG for the dashboard\n"
           "  -N       do not print OBJECT DETECTED lines\n"
           "  -q       quiet\n",
           p);
}
int main(int argc, char **argv)
{
    Options o;
    int ch;
    while ((ch = getopt(argc, argv, "u:Lm:t:W:H:X:F:k:j:R:P:o:s:i:n:rxqvwdSJNh")) != -1) {
        switch (ch) {
        case 'u': o.unit = atoi(optarg); break;
        case 'L': o.list_units = true; break;
        case 'm': o.model_dir = optarg; break;
        case 't': o.conf_th = (float)atof(optarg); break;
        case 'W': o.req_w = atoi(optarg); break;
        case 'H': o.req_h = atoi(optarg); break;
        case 'X': o.max_w = atoi(optarg); break;
        case 'F': o.hfov_deg = (float)atof(optarg); break;
        case 'k': o.corridor_deg = (float)atof(optarg); break;
        case 'j': o.threads = atoi(optarg); break;
        case 'R': o.runmask = (unsigned)strtoul(optarg, nullptr, 0); break;
        case 'P': o.prio = atoi(optarg); break;
        case 'o': o.out_path = optarg; break;
        case 's': o.save_period_s = (float)atof(optarg); break;
        case 'i': o.images.push_back(optarg); break;
        case 'n': o.max_frames = atol(optarg); break;
        case 'r': o.loop_images = true; break;
        case 'v': o.verbose = true; break;
        case 'w': o.no_winograd = true; break;
        case 'd': o.diagnose = true; break;
        case 'S': g_force_swap = true; break;
        case 'J': o.jpeg = false; break;
        case 'N': o.no_announce = true; break;
        case 'x': o.publish = false; break;
        case 'q': o.quiet = true; break;
        default: usage(argv[0]); return ch == 'h' ? 0 : 1;
        }
    }
    if (o.max_w < 64) o.max_w = 64;
    if (o.threads < 1) o.threads = 1;
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
#ifdef AEB_HAVE_CAMAPI
    if (o.list_units) {
        list_units();
        return 0;
    }
#endif
    set_low_priority_and_runmask(o);
    cv::setNumThreads(o.threads);
    printf("VIS: OpenCV %s, %d thread(s), runmask 0x%X, priority %d\n", CV_VERSION, cv::getNumThreads(),
           o.runmask, o.prio);
    if (!load_model(o)) return 1;
    if (o.publish && !shm_open_writer())
        fprintf(stderr, "VIS: continuing WITHOUT publishing to the AEB controller\n");
    if (!o.publish) o.jpeg = false;
    if (o.jpeg && !frame_shm_open_writer()) o.jpeg = false;
    if (!o.images.empty())
        snprintf(g_source, sizeof g_source, "%s", o.loop_images ? "image replay" : "image file");
    else
        snprintf(g_source, sizeof g_source, "camera unit %d", o.unit);
    int rc;
    if (!o.images.empty()) {
        rc = run_images(o);
    } else {
#ifdef AEB_HAVE_CAMAPI
        rc = run_camera(o);
#else
        fprintf(stderr, "VIS: built without libcamapi - use -i image.jpg\n");
        rc = 1;
#endif
    }
    shm_mark_dead();
    return rc;
}
