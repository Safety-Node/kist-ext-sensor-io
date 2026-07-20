// Live camera check (real RealSense needed). Captures real depth, runs
// it through the full pipeline — RVL encode -> serialize -> DDS loopback
// (lo) -> deserialize -> RVL decode — and shows original | decoded | diff
// colormapped. Lossless, so diff must stay black.
//
//   ./test_camera_view [frames] [out.png]
// With a display it opens a live window (ESC to quit); otherwise it
// captures a few frames and writes a PNG of the last one.

#include "realsense/transmitter/camera_capture.hpp"
#include "realsense/transmitter/depth_publisher.hpp"
#include "realsense/receiver/depth_receiver.hpp"
#include "realsense/transmitter/rvl_depth_encoder.hpp"
#include "realsense/receiver/rvl_depth_decoder.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

using namespace kist;

static cv::Mat colorize(const DepthFrame& d) {
    if (d.empty()) return cv::Mat::zeros(480, 640, CV_8UC3);
    cv::Mat depth16(d.height, d.width, CV_16UC1, const_cast<uint8_t*>(d.data.data()));
    cv::Mat norm8; depth16.convertTo(norm8, CV_8UC1, 255.0 / 4000.0);  // ~0..4m
    cv::Mat color; cv::applyColorMap(norm8, color, cv::COLORMAP_JET);
    color.setTo(cv::Scalar(0, 0, 0), depth16 == 0);
    return color;
}

int main(int argc, char** argv) {
    const int         frames   = (argc >= 2) ? std::atoi(argv[1]) : 150;
    const std::string out_path = (argc >= 3) ? argv[2] : "/tmp/camera_view.png";
    const bool        has_disp = [] { const char* d = std::getenv("DISPLAY"); return d && d[0]; }();

    CameraCapture cam;
    CameraCaptureConfig cfg;
    cfg.color_enabled = false;      // depth path only for now
    if (!cam.start(cfg)) return 1;

    auto& rx = DepthReceiver::instance();
    if (!rx.start(0, "lo")) return 1;
    DepthPublisher tx;
    if (!tx.start(0, "lo")) return 1;

    RvlDepthEncoder enc;
    RvlDepthDecoder dec;
    if (has_disp) cv::namedWindow("depth: original | decoded | diff", cv::WINDOW_AUTOSIZE);

    cv::Mat panel;
    int shown = 0;
    int64_t last_stamp = -1;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);

    while (shown < frames && std::chrono::steady_clock::now() < deadline) {
        auto cap = cam.depth_buf.GetData();
        if (!cap || cap->stamp_ns == last_stamp) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        last_stamp = cap->stamp_ns;

        // full pipeline: encode -> DDS loopback -> decode
        tx.publish(enc.encode(*cap));
        std::shared_ptr<const RvlDepthFrame> got;
        const auto rx_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
        do {
            got = rx.depth_buf.GetData();
        } while ((!got || got->stamp_ns != cap->stamp_ns) &&
                 std::chrono::steady_clock::now() < rx_deadline &&
                 (std::this_thread::sleep_for(std::chrono::milliseconds(1)), true));
        if (!got) continue;

        const DepthFrame decoded = dec.decode(*got);
        cv::Mat o16(cap->height, cap->width, CV_16UC1, const_cast<uint8_t*>(cap->data.data()));
        cv::Mat d16(decoded.height, decoded.width, CV_16UC1, const_cast<uint8_t*>(decoded.data.data()));
        cv::Mat diff; cv::absdiff(o16, d16, diff);
        double mx; cv::minMaxLoc(diff, nullptr, &mx);
        cv::Mat diff8; diff.convertTo(diff8, CV_8UC1, 50.0);
        cv::Mat diff_c; cv::cvtColor(diff8, diff_c, cv::COLOR_GRAY2BGR);

        cv::hconcat(std::vector<cv::Mat>{colorize(*cap), colorize(decoded), diff_c}, panel);
        ++shown;

        if (has_disp) {
            cv::imshow("depth: original | decoded | diff", panel);
            if (cv::waitKey(1) == 27) break;  // ESC
        } else if (shown % 30 == 0) {
            std::printf("frame %d  %dx%d  max_err=%.0f\n", shown, cap->width, cap->height, mx);
        }
    }

    if (!panel.empty()) {
        cv::imwrite(out_path, panel);
        std::printf("wrote %s (%d frames shown)\n", out_path.c_str(), shown);
    } else {
        std::fprintf(stderr, "no depth frames captured\n");
    }

    cam.stop();
    rx.stop();
    return panel.empty() ? 1 : 0;
}
