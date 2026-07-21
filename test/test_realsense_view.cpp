// Full-pipeline hardware test (real RealSense) of the rewired stack:
//   CameraTransmitter (capture -> H.264/RVL encode -> publish typed DDS)
//     -> lo -> CameraReceiver (receive typed DDS -> decode)
// Exercises the merged encode-publishers + idlc typed messages + receivers
// end to end with real frames. Shows color | depth (2x1); headless it
// writes /tmp/realsense_view.png each second.
//
//   ./test_realsense_view [width height]   (default 640 480)

#include "realsense/transmitter/camera_transmitter.hpp"
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
    CameraCaptureConfig ccfg;
    if (argc >= 3) {
        ccfg.depth_width = ccfg.color_width = std::atoi(argv[1]);
        ccfg.depth_height = ccfg.color_height = std::atoi(argv[2]);
    }
    ccfg.color_enabled = true;
    H264EncoderConfig ecfg;
    ecfg.width = ccfg.color_width; ecfg.height = ccfg.color_height; ecfg.fps = ccfg.color_fps;

    const bool has_disp = [] { const char* d = std::getenv("DISPLAY"); return d && d[0]; }();
    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    CameraReceiver rx;
    if (!rx.start(0, "lo")) return 1;
    CameraTransmitter tx;
    if (!tx.start(0, "lo", ccfg, ecfg)) return 1;

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
                if (cv::waitKey(1) == 27) break;
            }
        }
        if (!has_disp) std::this_thread::sleep_for(std::chrono::milliseconds(2));

        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            std::printf("received  color %2d fps  depth %2d fps  %s\n",
                        color_n, depth_n, (c && d) ? "" : "(waiting)");
            if (!has_disp && c && d) {
                cv::Mat cm(c->height, c->width, CV_8UC3, const_cast<uint8_t*>(c->data.data()));
                cv::Mat panel;
                cv::hconcat(std::vector<cv::Mat>{cm.clone(), colorize_depth(*d)}, panel);
                cv::imwrite("/tmp/realsense_view.png", panel);
            }
            color_n = depth_n = 0;
        }
    }

    tx.stop();
    rx.stop();
    return 0;
}
