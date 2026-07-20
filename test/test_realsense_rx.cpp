// Camera receiver — run on the LAN-connected machine (workstation):
//   ./test_realsense_rx [config_path]
// CameraReceiver: subscribe rt/kist/camera/{color,depth} -> decode ->
// show a 2x2 live view (color orig-decoded top, depth below). Headless,
// it writes /tmp/realsense_rx.png each second. Pair with
// test_realsense_tx on the machine with the RealSense.

#include "common/config.hpp"
#include "realsense/receiver/camera_receiver.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

using namespace kist;

static std::atomic<bool> g_stop{false};

static cv::Mat colorize_depth(const DepthFrame& d) {
    cv::Mat depth16(d.height, d.width, CV_16UC1, const_cast<uint8_t*>(d.data.data()));
    cv::Mat n8; depth16.convertTo(n8, CV_8UC1, 255.0 / 4000.0);
    cv::Mat c; cv::applyColorMap(n8, c, cv::COLORMAP_JET);
    c.setTo(cv::Scalar(0, 0, 0), depth16 == 0);
    return c;
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

    CameraReceiver rx;
    if (!rx.start(domain, interface)) return 1;

    int color_n = 0, depth_n = 0;
    int64_t last_color = -1, last_depth = -1;
    auto window = std::chrono::steady_clock::now();

    while (!g_stop) {
        auto c = rx.color().GetData();
        auto d = rx.depth().GetData();
        if (c && c->stamp_ns != last_color) { last_color = c->stamp_ns; ++color_n; }
        if (d && d->stamp_ns != last_depth) { last_depth = d->stamp_ns; ++depth_n; }

        if (has_disp && c && d) {
            cv::Mat cm(c->height, c->width, CV_8UC3, const_cast<uint8_t*>(c->data.data()));
            cv::Mat dm = colorize_depth(*d);
            cv::Mat both;
            if (cm.rows == dm.rows)
                cv::hconcat(std::vector<cv::Mat>{cm.clone(), dm}, both);
            else
                both = cm.clone();
            cv::imshow("kist camera rx: color | depth", both);
            if (cv::waitKey(1) == 27) break;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            std::printf("color %2d fps  depth %2d fps  %s\n", color_n, depth_n,
                        (c && d) ? "" : "(waiting)");
            if (!has_disp && c && d) {
                cv::Mat cm(c->height, c->width, CV_8UC3, const_cast<uint8_t*>(c->data.data()));
                cv::Mat both;
                cv::hconcat(std::vector<cv::Mat>{cm.clone(), colorize_depth(*d)}, both);
                cv::imwrite("/tmp/realsense_rx.png", both);
            }
            color_n = depth_n = 0;
        }
    }

    rx.stop();
    return 0;
}
