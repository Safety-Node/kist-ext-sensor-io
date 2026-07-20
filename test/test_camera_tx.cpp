// Camera transmitter probe — run on the machine with the RealSense
// (the robot side):
//   ./test_camera_tx [config_path]
// Captures depth -> RVL encode -> publish rt/kist/camera/depth over the
// interface in config/config.yaml. Prints publish rate + compression
// once per second. Ctrl-C to stop. Pair with test_camera_rx on the
// LAN-connected machine.

#include "common/config.hpp"
#include "realsense/transmitter/camera_capture.hpp"
#include "realsense/transmitter/rvl_depth_encoder.hpp"
#include "realsense/transmitter/depth_publisher.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(config_path);
    const auto u = Config::instance().root()["unitree"];
    const int         domain    = u["domain_id"].as<int>();
    const std::string interface = u["network_interface"].as<std::string>();

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    CameraCapture cam;
    CameraCaptureConfig cfg;
    cfg.color_enabled  = false;            // depth-only probe: no color -> no align
    cfg.align_to_color = false;
    if (!cam.start(cfg)) return 1;

    DepthPublisher pub;
    if (!pub.start(domain, interface)) return 1;

    RvlDepthEncoder enc;
    int64_t  last_stamp = -1;
    uint64_t last_seq = 0;
    bool     have_seq = false;
    int      published = 0;
    int      dropped = 0;   // capture frames the polling missed (seq gaps)
    std::size_t last_raw = 0, last_rvl = 0;
    auto window = std::chrono::steady_clock::now();

    while (!g_stop) {
        auto cap = cam.depth_buf.GetData();
        if (!cap || cap->stamp_ns == last_stamp) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        } else {
            last_stamp = cap->stamp_ns;
            if (have_seq && cap->sequence > last_seq + 1)
                dropped += int(cap->sequence - last_seq - 1);
            last_seq = cap->sequence;
            have_seq = true;
            RvlDepthFrame rvl = enc.encode(*cap);
            last_raw = cap->data.size();
            last_rvl = rvl.data.size();
            pub.publish(rvl);
            ++published;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            if (published > 0)
                std::printf("published %2d fps  dropped %d (poll-missed)  %zu->%zu bytes (%.1fx)\n",
                            published, dropped, last_raw, last_rvl,
                            last_rvl ? double(last_raw) / last_rvl : 0.0);
            else
                std::printf("no depth frames\n");
            published = 0;
            dropped = 0;
        }
    }

    cam.stop();
    return 0;
}
