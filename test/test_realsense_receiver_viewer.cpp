// Visual RealsenseReceiver runner — subscribes + decodes, then shows
// color | depth (2x1). Same as test_realsense_receiver but with a viewer:
// with a DISPLAY it opens a window; headless it writes /tmp/realsense_view.png
// once per second. Pair with test_realsense_transmitter.
//   ./test_realsense_receiver_viewer [config_path]   (default config/config.yaml)
// Ctrl-C / SIGTERM (or ESC in the window) to stop.

#include "system/realsense_receiver.hpp"
#include "common/config.hpp"

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
    const auto& root = Config::instance().root();
    const auto unitree      = root["unitree"];
    const int domain_id     = unitree["domain_id"].as<int>(0);
    const std::string iface = unitree["network_interface"].as<std::string>("lo");

    const bool has_disp = [] { const char* d = std::getenv("DISPLAY"); return d && d[0]; }();

    RealsenseReceiver rx;
    if (!rx.start(domain_id, iface))
        return 1;

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    std::printf("[test_realsense_receiver_viewer] subscribing on domain=%d iface=%s (%s)\n",
                domain_id, iface.c_str(),
                has_disp ? "window" : "headless -> /tmp/realsense_view.png");

    int color_n = 0, depth_n = 0;
    int64_t last_c = -1, last_d = -1;
    auto window = std::chrono::steady_clock::now();

    while (!g_stop) {
        auto c = rx.color().GetData();
        auto d = rx.depth().GetData();
        if (c && c->stamp_ns != last_c) { last_c = c->stamp_ns; ++color_n; }
        if (d && d->stamp_ns != last_d) { last_d = d->stamp_ns; ++depth_n; }

        if (c && d) {
            cv::Mat cm(c->height, c->width, CV_8UC3, const_cast<uint8_t*>(c->data.data()));
            cv::Mat panel;
            if (cm.rows == d->height)
                cv::hconcat(std::vector<cv::Mat>{cm.clone(), colorize_depth(*d)}, panel);
            else
                panel = cm.clone();
            if (has_disp) {
                cv::imshow("realsense rx: color (H.264) | depth (RVL)", panel);
                if (cv::waitKey(1) == 27) break;  // ESC
            }
        }
        if (!has_disp) std::this_thread::sleep_for(std::chrono::milliseconds(2));

        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            std::printf("received  color %2d fps  depth %2d fps%s\n",
                        color_n, depth_n, (c && d) ? "" : "  (waiting)");
            if (!has_disp && c && d) {
                cv::Mat cm(c->height, c->width, CV_8UC3, const_cast<uint8_t*>(c->data.data()));
                cv::Mat panel;
                cv::hconcat(std::vector<cv::Mat>{cm.clone(), colorize_depth(*d)}, panel);
                cv::imwrite("/tmp/realsense_view.png", panel);
            }
            color_n = depth_n = 0;
        }
    }

    rx.stop();
    return 0;
}
