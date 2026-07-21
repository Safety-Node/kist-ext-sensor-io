// CameraCapture-only probe (real RealSense, no DDS/encode/OpenCV).
// Confirms the capture thread fills BOTH color_buf and depth_buf and at
// what rate — the most isolated camera test.
//
//   ./test_camera_capture [width height]   (default 640 480)
// Prints, once per second, each buffer's dimensions and new-frame rate.
// Ctrl-C to stop.

#include "realsense/transmitter/camera_capture.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    CameraCaptureConfig cfg;
    if (argc >= 3) {
        cfg.depth_width = cfg.color_width = std::atoi(argv[1]);
        cfg.depth_height = cfg.color_height = std::atoi(argv[2]);
    }
    cfg.color_enabled  = true;   // exercise both streams
    cfg.align_to_color = true;   // capture + align (isolates align cost)

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    std::printf("[probe] requested %dx%d  color=%s  align=%s\n",
                cfg.depth_width, cfg.depth_height,
                cfg.color_enabled ? "on" : "off",
                cfg.align_to_color ? "ON" : "off");

    CameraCapture cam;
    if (!cam.start(cfg))
        return 1;

    int64_t last_color = -1, last_depth = -1;
    int     color_n = 0, depth_n = 0;
    auto    window = std::chrono::steady_clock::now();

    while (!g_stop) {
        auto c = cam.color_buf.GetData();
        auto d = cam.depth_buf.GetData();
        if (c && c->stamp_ns != last_color) { last_color = c->stamp_ns; ++color_n; }
        if (d && d->stamp_ns != last_depth) { last_depth = d->stamp_ns; ++depth_n; }

        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            if (c)
                std::printf("color %dx%d stride=%d  %2d fps   ",
                            c->width, c->height, c->stride_bytes, color_n);
            else
                std::printf("color (empty)            ");
            if (d)
                std::printf("| depth %dx%d stride=%d scale=%.4f  %2d fps\n",
                            d->width, d->height, d->stride_bytes, d->depth_scale, depth_n);
            else
                std::printf("| depth (empty)\n");
            color_n = depth_n = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    cam.stop();
    return 0;
}
