// Capture + encode isolation probe (real RealSense, NO DDS/publish).
// Replicates the real Tx thread layout minus the wire: T1 CameraCapture,
// plus a color H.264-encode thread and a depth RVL-encode thread that
// pull from the capture buffers and encode — but never publish.
//
// Purpose: separate "encode CPU / core contention" from "DDS/publish".
// It counts, per stream, frames the CAPTURE produced (sequence) vs frames
// each encoder actually consumed, and flags capture-level drops (seq
// gaps) — i.e. whether the encoders starve the capture thread.
//
//   ./test_camera_encode [width height] [noalign]   (default 640 480, align on)
// Ctrl-C to stop.

#include "realsense/transmitter/camera_capture.hpp"
#include "realsense/transmitter/h264_encoder.hpp"
#include "realsense/transmitter/rvl_encoder.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
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
    cfg.color_enabled  = true;
    cfg.align_to_color = !(argc >= 4 && std::string(argv[3]) == "noalign");

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    std::printf("[probe] %dx%d  align=%s  (capture + encode, NO DDS)\n",
                cfg.depth_width, cfg.depth_height, cfg.align_to_color ? "ON" : "off");

    CameraCapture cam;
    if (!cam.start(cfg)) return 1;

    // capture-level drop counters (sequence gaps = librealsense dropped)
    std::atomic<int> color_enc{0}, depth_enc{0};
    std::atomic<int> color_cap_drop{0}, depth_cap_drop{0};

    // T2: color H.264 encode (no publish)
    std::thread color_thr([&] {
        H264EncoderConfig ec; ec.width = cfg.color_width; ec.height = cfg.color_height;
        H264Encoder enc(ec);
        int64_t last_stamp = -1; uint64_t last_seq = 0; bool have = false;
        while (!g_stop) {
            auto f = cam.color_buf.GetData();
            if (f && f->stamp_ns != last_stamp) {
                last_stamp = f->stamp_ns;
                if (have && f->sequence > last_seq + 1) color_cap_drop += int(f->sequence - last_seq - 1);
                last_seq = f->sequence; have = true;
                if (enc.encode(*f)) ++color_enc;
            } else std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // T3: depth RVL encode (no publish)
    std::thread depth_thr([&] {
        RvlEncoder enc;
        int64_t last_stamp = -1; uint64_t last_seq = 0; bool have = false;
        while (!g_stop) {
            auto f = cam.depth_buf.GetData();
            if (f && f->stamp_ns != last_stamp) {
                last_stamp = f->stamp_ns;
                if (have && f->sequence > last_seq + 1) depth_cap_drop += int(f->sequence - last_seq - 1);
                last_seq = f->sequence; have = true;
                enc.encode(*f); ++depth_enc;
            } else std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::printf("color: enc %2d fps  cap-drop %d   | depth: enc %2d fps  cap-drop %d\n",
                    color_enc.exchange(0), color_cap_drop.exchange(0),
                    depth_enc.exchange(0), depth_cap_drop.exchange(0));
    }

    color_thr.join();
    depth_thr.join();
    cam.stop();
    return 0;
}
