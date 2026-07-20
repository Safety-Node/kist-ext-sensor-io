// DDS loopback probe (no hardware): ColorPublisher -> lo -> ColorReceiver
// in one process. Proves the compressed-color wire path (the
// VoxelMapCompressed_ carrier, serialization, QoS, hook), then H.264
// decodes the received frame and checks it resembles the original
// (lossy, so closeness — not byte equality).

#include "realsense/transmitter/h264_color_encoder.hpp"
#include "realsense/transmitter/color_publisher.hpp"
#include "realsense/receiver/h264_color_decoder.hpp"
#include "realsense/receiver/color_receiver.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

using namespace kist;

static int g_failures = 0;
static void check(const char* name, bool ok) {
    std::printf("%-46s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

static ColorFrame make_color(int w, int h) {
    ColorFrame f;
    f.width = w; f.height = h; f.stride_bytes = w * 3;
    f.sequence = 3; f.stamp_ns = 555; f.frame_id = "camera_color";
    f.data.resize(static_cast<size_t>(w) * h * 3);
    for (int r = 0; r < h; ++r)
        for (int c = 0; c < w; ++c) {
            uint8_t* p = f.data.data() + (r * w + c) * 3;
            p[0] = uint8_t(c % 256); p[1] = uint8_t(r % 256); p[2] = uint8_t((r + c) % 256);
        }
    return f;
}

int main() {
    const int w = 640, h = 480;
    H264EncoderConfig cfg; cfg.width = w; cfg.height = h;
    const ColorFrame     original   = make_color(w, h);
    const H264ColorFrame compressed = *H264ColorEncoder{cfg}.encode(original);

    std::atomic<int> hook_calls{0};
    auto& rx = ColorReceiver::instance();
    rx.set_on_frame([&](const H264ColorFrame&) { ++hook_calls; });
    if (!rx.start(0, "lo")) { std::printf("FAILED to start receiver\n"); return 1; }

    ColorPublisher tx;
    if (!tx.start(0, "lo")) { std::printf("FAILED to start publisher\n"); return 1; }

    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::seconds(5);
    while (clock::now() < deadline && !rx.color_buf.GetData()) {
        tx.publish(compressed);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    auto got = rx.color_buf.GetData();
    check("compressed color received over DDS", got != nullptr);
    if (got) {
        check("metadata survived the wire",
              got->width == w && got->height == h && got->sequence == 3 &&
              got->is_keyframe && got->frame_id == "camera_color");
        check("H.264 payload identical", got->data == compressed.data);

        auto decoded = H264ColorDecoder{}.decode(*got);
        bool close = false;
        if (decoded && decoded->width == w && decoded->height == h) {
            double acc = 0;
            for (size_t i = 0; i < decoded->data.size(); ++i)
                acc += std::abs(int(decoded->data[i]) - int(original.data[i]));
            close = (acc / decoded->data.size()) < 8.0;
        }
        check("decodes end-to-end, close to original (lossy)", close);
    }
    check("set_on_frame hook fired", hook_calls > 0);

    rx.stop();
    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
