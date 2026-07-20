// Full-pipeline visual check of BOTH streams (real RealSense). Captures
// color+depth, runs color through H.264 and depth through RVL, each over
// a DDS loopback (lo):
//   capture -> encode -> publish -> DDS -> receive -> decode -> display
// Shows a 2x2 panel: color original | H.264 decoded (top),
// depth original | RVL decoded (bottom). Depth is lossless (bottom two
// identical); color is lossy but near-identical.
//
//   ./test_realsense_view [frames] [out.png]

#include "realsense/transmitter/camera_capture.hpp"
#include "realsense/transmitter/h264_color_encoder.hpp"
#include "realsense/transmitter/color_publisher.hpp"
#include "realsense/transmitter/rvl_depth_encoder.hpp"
#include "realsense/transmitter/depth_publisher.hpp"
#include "realsense/receiver/h264_color_decoder.hpp"
#include "realsense/receiver/color_receiver.hpp"
#include "realsense/receiver/rvl_depth_decoder.hpp"
#include "realsense/receiver/depth_receiver.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

using namespace kist;

static cv::Mat colorize_depth(const DepthFrame& d) {
    cv::Mat depth16(d.height, d.width, CV_16UC1, const_cast<uint8_t*>(d.data.data()));
    cv::Mat n8; depth16.convertTo(n8, CV_8UC1, 255.0 / 4000.0);
    cv::Mat c; cv::applyColorMap(n8, c, cv::COLORMAP_JET);
    c.setTo(cv::Scalar(0, 0, 0), depth16 == 0);
    return c;
}

static cv::Mat wait_new_depth(DepthReceiver& rx, int64_t stamp) {
    const auto dl = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < dl) {
        auto g = rx.depth_buf.GetData();
        if (g && g->stamp_ns == stamp) return colorize_depth(RvlDepthDecoder{}.decode(*g));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return {};
}

int main(int argc, char** argv) {
    const int         frames   = (argc >= 2) ? std::atoi(argv[1]) : 120;
    const std::string out_path = (argc >= 3) ? argv[2] : "/tmp/realsense_view.png";
    const bool has_disp = [] { const char* d = std::getenv("DISPLAY"); return d && d[0]; }();

    CameraCapture cam;
    CameraCaptureConfig cfg;  // color + depth, aligned (real config)
    if (!cam.start(cfg)) return 1;

    auto& crx = ColorReceiver::instance();
    auto& drx = DepthReceiver::instance();
    if (!crx.start(0, "lo") || !drx.start(0, "lo")) return 1;
    ColorPublisher ctx; DepthPublisher dtx;
    if (!ctx.start(0, "lo") || !dtx.start(0, "lo")) return 1;

    H264EncoderConfig ecfg;
    ecfg.width = cfg.color_width; ecfg.height = cfg.color_height; ecfg.fps = cfg.color_fps;
    H264ColorEncoder cenc(ecfg);
    H264ColorDecoder cdec;
    RvlDepthEncoder  denc;

    cv::Mat panel;
    int shown = 0;
    int64_t last_color_stamp = -1;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(40);

    while (shown < frames && std::chrono::steady_clock::now() < deadline) {
        auto color = cam.color_buf.GetData();
        auto depth = cam.depth_buf.GetData();
        if (!color || !depth || color->stamp_ns == last_color_stamp) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        last_color_stamp = color->stamp_ns;

        // color: H.264 encode -> publish -> receive -> decode
        auto cenc_f = cenc.encode(*color);
        if (!cenc_f) continue;
        ctx.publish(*cenc_f);

        // depth: RVL encode -> publish
        dtx.publish(denc.encode(*depth));

        // receive both back
        std::shared_ptr<const H264ColorFrame> crx_f;
        const auto dl = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
        while (std::chrono::steady_clock::now() < dl) {
            crx_f = crx.color_buf.GetData();
            if (crx_f && crx_f->stamp_ns == color->stamp_ns) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        auto cdec_f = (crx_f) ? cdec.decode(*crx_f) : std::nullopt;
        cv::Mat depth_dec = wait_new_depth(drx, depth->stamp_ns);
        if (!cdec_f || depth_dec.empty()) continue;

        // build 2x2 panel
        cv::Mat c_orig(color->height, color->width, CV_8UC3, const_cast<uint8_t*>(color->data.data()));
        cv::Mat c_dec(cdec_f->height, cdec_f->width, CV_8UC3, cdec_f->data.data());
        cv::Mat d_orig = colorize_depth(*depth);
        cv::Mat top, bot;
        cv::hconcat(std::vector<cv::Mat>{c_orig.clone(), c_dec.clone()}, top);
        cv::hconcat(std::vector<cv::Mat>{d_orig, depth_dec}, bot);
        cv::putText(top, "color original", {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {255,255,255}, 2);
        cv::putText(top, "color H.264", {top.cols/2+10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {255,255,255}, 2);
        cv::putText(bot, "depth original", {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {255,255,255}, 2);
        cv::putText(bot, "depth RVL", {bot.cols/2+10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {255,255,255}, 2);
        cv::vconcat(std::vector<cv::Mat>{top, bot}, panel);
        ++shown;

        if (has_disp) {
            cv::imshow("realsense: color (H.264) + depth (RVL) over DDS", panel);
            if (cv::waitKey(1) == 27) break;
        } else if (shown % 30 == 0) {
            std::printf("frame %d  color %zuB  \n", shown, cenc_f->data.size());
        }
    }

    if (!panel.empty()) { cv::imwrite(out_path, panel); std::printf("wrote %s (%d frames)\n", out_path.c_str(), shown); }
    else std::fprintf(stderr, "no frames\n");

    cam.stop(); crx.stop(); drx.stop();
    return panel.empty() ? 1 : 0;
}
