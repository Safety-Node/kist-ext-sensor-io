// Visual check of the depth path (OpenCV — probe only, not part of the
// library). Builds a recognizable synthetic depth scene, runs it through
// the REAL pipeline — RVL encode -> serialize -> DDS loopback (lo) ->
// deserialize -> RVL decode — then shows original | decoded | diff as
// colormapped images. Lossless, so the diff panel must be all black.
//
//   ./test_depth_view [out.png]
// Writes a PNG (default /tmp/depth_view.png) and, if a display is
// available, opens a window (press any key to close).

#include "camera/depth_publisher.hpp"
#include "camera/depth_receiver.hpp"
#include "camera/rvl_depth_codec.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

using namespace kist;

// Synthetic 16-bit depth scene: a smooth background ramp with a nearer
// slanted bar and a hole of invalid (zero) depth — exercises zero runs,
// smooth deltas, and a sharp edge, and is easy to recognize by eye.
static DepthFrame make_scene(int w, int h) {
    std::vector<uint16_t> px(static_cast<size_t>(w) * h);
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            uint16_t v = static_cast<uint16_t>(800 + (r + c));   // background ramp
            if (r > h / 3 && r < 2 * h / 3 && c > (r % 40))       // slanted near bar
                v = 400;
            if ((c - w / 2) * (c - w / 2) + (r - h / 2) * (r - h / 2) < 900)
                v = 0;                                            // invalid hole
            px[r * w + c] = v;
        }
    }
    DepthFrame f;
    f.width = w; f.height = h; f.stride_bytes = w * 2;
    f.sequence = 1; f.stamp_ns = 42; f.frame_id = "camera_depth";
    f.data.resize(static_cast<size_t>(w) * h * 2);
    std::memcpy(f.data.data(), px.data(), f.data.size());
    return f;
}

// 16-bit depth -> 8-bit colormap (invalid/zero shown black).
static cv::Mat colorize(const DepthFrame& d) {
    cv::Mat depth16(d.height, d.width, CV_16UC1,
                    const_cast<uint8_t*>(d.data.data()));
    cv::Mat norm8;
    depth16.convertTo(norm8, CV_8UC1, 255.0 / 2000.0);  // ~0..2m window
    cv::Mat color;
    cv::applyColorMap(norm8, color, cv::COLORMAP_JET);
    color.setTo(cv::Scalar(0, 0, 0), depth16 == 0);     // holes -> black
    return color;
}

int main(int argc, char** argv) {
    const std::string out_path = (argc >= 2) ? argv[1] : "/tmp/depth_view.png";
    const int w = 640, h = 480;

    const DepthFrame     original   = make_scene(w, h);
    const RvlDepthFrame  compressed = RvlDepthEncoder{}.encode(original);

    // Real DDS round-trip over loopback.
    auto& rx = DepthReceiver::instance();
    if (!rx.start(0, "lo")) return 1;
    DepthPublisher tx;
    if (!tx.start(0, "lo")) return 1;

    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::seconds(5);
    while (clock::now() < deadline && !rx.depth_buf.GetData()) {
        tx.publish(compressed);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    auto got = rx.depth_buf.GetData();
    if (!got) { std::fprintf(stderr, "no frame received\n"); rx.stop(); return 1; }

    const DepthFrame decoded = RvlDepthDecoder{}.decode(*got);
    rx.stop();

    // Panels: original | decoded | diff (x50 so any error is visible).
    cv::Mat orig_c = colorize(original);
    cv::Mat dec_c  = colorize(decoded);
    cv::Mat o16(h, w, CV_16UC1, const_cast<uint8_t*>(original.data.data()));
    cv::Mat d16(h, w, CV_16UC1, const_cast<uint8_t*>(decoded.data.data()));
    cv::Mat diff; cv::absdiff(o16, d16, diff);
    cv::Mat diff8; diff.convertTo(diff8, CV_8UC1, 50.0);
    cv::Mat diff_c; cv::cvtColor(diff8, diff_c, cv::COLOR_GRAY2BGR);

    auto label = [](cv::Mat& m, const std::string& t) {
        cv::putText(m, t, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    {255, 255, 255}, 2);
    };
    label(orig_c, "original");
    label(dec_c, "decoded (RVL+DDS)");
    label(diff_c, "diff x50 (black=lossless)");

    cv::Mat panel;
    cv::hconcat(std::vector<cv::Mat>{orig_c, dec_c, diff_c}, panel);

    const double max_err = [&] { double mn, mx; cv::minMaxLoc(diff, &mn, &mx); return mx; }();
    std::printf("RVL %d->%d bytes (%.1fx), max pixel error = %.0f  %s\n",
                int(original.data.size()), int(compressed.data.size()),
                double(original.data.size()) / compressed.data.size(),
                max_err, max_err == 0 ? "(lossless)" : "(!!)");

    cv::imwrite(out_path, panel);
    std::printf("wrote %s\n", out_path.c_str());

    if (const char* disp = std::getenv("DISPLAY"); disp && disp[0]) {
        cv::imshow("depth: original | decoded | diff", panel);
        std::printf("showing window — press any key to close\n");
        cv::waitKey(0);
    }
    return max_err == 0 ? 0 : 1;
}
