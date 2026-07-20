// Camera receiver probe — run on the LAN-connected machine (the
// workstation):
//   ./test_camera_rx [config_path]
// Subscribes to rt/kist/camera/depth over the interface in
// config/config.yaml, RVL-decodes, and shows a live depth colormap
// (ESC to quit); headless, it writes /tmp/camera_rx.png each second.
// Pair with test_camera_tx on the machine with the RealSense.

#include "common/config.hpp"
#include "realsense/receiver/depth_receiver.hpp"
#include "realsense/receiver/rvl_depth_decoder.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

static cv::Mat colorize(const DepthFrame& d) {
    cv::Mat depth16(d.height, d.width, CV_16UC1, const_cast<uint8_t*>(d.data.data()));
    cv::Mat norm8; depth16.convertTo(norm8, CV_8UC1, 255.0 / 4000.0);  // ~0..4m
    cv::Mat color; cv::applyColorMap(norm8, color, cv::COLORMAP_JET);
    color.setTo(cv::Scalar(0, 0, 0), depth16 == 0);
    return color;
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(config_path);
    const auto u = Config::instance().root()["unitree"];
    const int         domain    = u["domain_id"].as<int>();
    const std::string interface = u["network_interface"].as<std::string>();
    const bool has_disp = [] { const char* d = std::getenv("DISPLAY"); return d && d[0]; }();

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    auto& rx = DepthReceiver::instance();
    if (!rx.start(domain, interface)) return 1;

    RvlDepthDecoder dec;
    int64_t last_stamp = -1;
    int     received = 0;
    auto    window = std::chrono::steady_clock::now();

    while (!g_stop) {
        auto got = rx.depth_buf.GetDataWithTime();
        if (got.HasData() && got.data->stamp_ns != last_stamp) {
            last_stamp = got.data->stamp_ns;
            ++received;
            const DepthFrame d = dec.decode(*got.data);
            cv::Mat view = colorize(d);
            if (has_disp) {
                cv::imshow("kist camera depth (rx)", view);
                if (cv::waitKey(1) == 27) break;  // ESC
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            auto snap = rx.depth_buf.GetData();
            if (snap) {
                std::printf("received %2d fps  %dx%d  age=%.0fms\n",
                            received, snap->width, snap->height,
                            rx.depth_buf.GetDataWithTime().GetAgeMs());
                if (!has_disp)
                    cv::imwrite("/tmp/camera_rx.png", colorize(dec.decode(*snap)));
            } else {
                std::printf("no depth (buffer empty)\n");
            }
            received = 0;
        }
    }

    rx.stop();
    return 0;
}
