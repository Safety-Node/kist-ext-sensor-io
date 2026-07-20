// Live color check (real RealSense). Captures real BGR color, runs it
// through H.264 encode -> decode, and shows original | decoded side by
// side. H.264 is lossy, so they should look near-identical (not exact).
//
//   ./test_color_view [frames] [out.png]

#include "realsense/transmitter/camera_capture.hpp"
#include "realsense/transmitter/h264_color_encoder.hpp"
#include "realsense/receiver/h264_color_decoder.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

using namespace kist;

int main(int argc, char** argv) {
    const int         frames   = (argc >= 2) ? std::atoi(argv[1]) : 90;
    const std::string out_path = (argc >= 3) ? argv[2] : "/tmp/color_view.png";
    const bool has_disp = [] { const char* d = std::getenv("DISPLAY"); return d && d[0]; }();

    CameraCapture cam;
    CameraCaptureConfig cfg;         // color on, depth on (real config)
    if (!cam.start(cfg)) return 1;

    H264EncoderConfig ecfg;
    ecfg.width = cfg.color_width; ecfg.height = cfg.color_height; ecfg.fps = cfg.color_fps;
    H264ColorEncoder enc(ecfg);
    H264ColorDecoder dec;

    cv::Mat panel;
    int shown = 0;
    int64_t last_stamp = -1;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);

    while (shown < frames && std::chrono::steady_clock::now() < deadline) {
        auto cap = cam.color_buf.GetData();
        if (!cap || cap->stamp_ns == last_stamp) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        last_stamp = cap->stamp_ns;

        auto enc_frame = enc.encode(*cap);
        if (!enc_frame) continue;
        auto dec_frame = dec.decode(*enc_frame);
        if (!dec_frame) continue;      // decoder warming up (pre-keyframe)

        cv::Mat orig(cap->height, cap->width, CV_8UC3, const_cast<uint8_t*>(cap->data.data()));
        cv::Mat deco(dec_frame->height, dec_frame->width, CV_8UC3, const_cast<uint8_t*>(dec_frame->data.data()));
        cv::Mat o = orig.clone(), d = deco.clone();
        cv::putText(o, "original", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {255,255,255}, 2);
        cv::putText(d, "H.264 decoded", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {255,255,255}, 2);
        cv::hconcat(std::vector<cv::Mat>{o, d}, panel);
        ++shown;

        if (has_disp) {
            cv::imshow("color: original | H.264 decoded", panel);
            if (cv::waitKey(1) == 27) break;
        } else if (shown % 30 == 0) {
            std::printf("frame %d  %dx%d  %zu bytes\n",
                        shown, cap->width, cap->height, enc_frame->data.size());
        }
    }

    if (!panel.empty()) {
        cv::imwrite(out_path, panel);
        std::printf("wrote %s (%d frames)\n", out_path.c_str(), shown);
    } else {
        std::fprintf(stderr, "no color frames captured\n");
    }
    cam.stop();
    return panel.empty() ? 1 : 0;
}
