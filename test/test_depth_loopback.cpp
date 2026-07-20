// DDS loopback probe (no hardware): DepthPublisher -> lo -> DepthReceiver
// in one process. Proves the full compressed-depth wire path — the
// VoxelMapCompressed_ byte carrier, serialization, QoS, and the hook —
// then RVL-decodes the received frame and checks it against the original
// (lossless end to end).

#include "realsense/transmitter/depth_publisher.hpp"
#include "realsense/receiver/depth_receiver.hpp"
#include "realsense/transmitter/rvl_depth_encoder.hpp"
#include "realsense/receiver/rvl_depth_decoder.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

using namespace kist;

static int g_failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%-46s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

int main() {
    // Build a raw depth frame, RVL-encode it — this is what a capture
    // thread would hand the publisher.
    const int w = 64, h = 48;
    std::vector<uint16_t> px(static_cast<size_t>(w) * h);
    for (int r = 0; r < h; ++r)
        for (int c = 0; c < w; ++c)
            px[r * w + c] = static_cast<uint16_t>((c % 8 == 0) ? 0 : 1000 + r + c);

    DepthFrame raw;
    raw.width = w; raw.height = h; raw.stride_bytes = w * 2;
    raw.sequence = 99; raw.stamp_ns = 111222333; raw.frame_id = "camera_depth";
    raw.data.resize(static_cast<size_t>(w) * h * 2);
    std::memcpy(raw.data.data(), px.data(), raw.data.size());

    const RvlDepthFrame compressed = RvlDepthEncoder{}.encode(raw);

    std::atomic<int> hook_calls{0};
    auto& rx = DepthReceiver::instance();
    rx.set_on_frame([&](const RvlDepthFrame&) { ++hook_calls; });
    if (!rx.start(0, "lo")) { std::printf("FAILED to start receiver\n"); return 1; }

    DepthPublisher tx;
    if (!tx.start(0, "lo")) { std::printf("FAILED to start publisher\n"); return 1; }

    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::seconds(5);
    while (clock::now() < deadline && !rx.depth_buf.GetData()) {
        tx.publish(compressed);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    auto got = rx.depth_buf.GetData();
    check("compressed frame received over DDS", got != nullptr);
    if (got) {
        check("metadata survived the wire",
              got->width == w && got->height == h && got->sequence == 99 &&
              got->stamp_ns == 111222333 && got->frame_id == "camera_depth");
        check("RVL payload identical", got->data == compressed.data);

        // decode end-to-end and compare to the original pixels
        const DepthFrame back = RvlDepthDecoder{}.decode(*got);
        const auto* bp = reinterpret_cast<const uint16_t*>(back.data.data());
        bool pixels_ok = back.width == w && back.height == h;
        for (size_t i = 0; pixels_ok && i < px.size(); ++i)
            pixels_ok = (bp[i] == px[i]);
        check("end-to-end lossless (decoded == original)", pixels_ok);
    }
    check("set_on_frame hook fired", hook_calls > 0);

    rx.stop();
    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
