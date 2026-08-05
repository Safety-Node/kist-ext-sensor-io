// Visual multi-camera RealsenseReceiver runner — subscribes to every camera in
// the `cameras` list (or one named via argv[2]) and shows them in one window:
// a row of color tiles on top, the matching depth tiles below (one column per
// camera). Each tile is labeled with its camera name + received fps; a stale or
// absent camera shows a "no signal" tile.
//   ./test_realsense_receiver_viewer [config_path] [camera_name]
// With a DISPLAY it opens a window (ESC to quit); headless it writes
// /tmp/realsense_view.png once per second.

#include "system/realsense_receiver.hpp"
#include "realsense/realsense_config.hpp"
#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace kist;

static std::atomic<bool> g_stop{false};

namespace {
constexpr int    kTileW = 640, kTileH = 480;   // per-camera tile size (native)
constexpr double kDepthRangeM = 4.0;   // depth colormap span: 0..4 m -> JET
const cv::Scalar kBg(40, 40, 40), kText(230, 230, 230);

struct CamRx {
    std::string name;
    std::unique_ptr<RealsenseReceiver> rx;
    int64_t last_c = -1, last_d = -1;
    int     color_n = 0, depth_n = 0;        // counters (this second)
    int     color_fps = 0, depth_fps = 0;    // latched once/sec for the label
};

// Depth -> colorized tile. Resize the raw 16-bit depth to tile size first so we
// only colormap tile-many pixels (cheap vs full-res).
cv::Mat depth_tile(const DepthFrame& d) {
    cv::Mat depth16(d.height, d.width, CV_16UC1, const_cast<uint8_t*>(d.data.data()));
    cv::Mat small; cv::resize(depth16, small, cv::Size(kTileW, kTileH), 0, 0, cv::INTER_NEAREST);
    // Normalize by real distance (via depth_scale) so cameras with different
    // scales (D435 1mm vs D405 0.1mm) share one 0..kDepthRangeM color range.
    const double ds = (d.depth_scale > 0.f) ? d.depth_scale : 0.001;
    cv::Mat n8; small.convertTo(n8, CV_8UC1, 255.0 * ds / kDepthRangeM);
    cv::Mat c; cv::applyColorMap(n8, c, cv::COLORMAP_JET);
    c.setTo(cv::Scalar(0, 0, 0), small == 0);
    return c;
}

// `content` fitted into a kTileW x kTileH frame (or "no signal"), title-barred.
cv::Mat labeled_tile(const cv::Mat& content, const std::string& label) {
    cv::Mat tile(kTileH, kTileW, CV_8UC3, kBg);
    if (!content.empty())
        cv::resize(content, tile, cv::Size(kTileW, kTileH));
    else
        cv::putText(tile, "no signal", {kTileW / 2 - 60, kTileH / 2},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, kText, 1, cv::LINE_AA);
    cv::rectangle(tile, {0, 0}, {kTileW, 20}, cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(tile, label, {6, 15}, cv::FONT_HERSHEY_SIMPLEX, 0.42, kText, 1, cv::LINE_AA);
    return tile;
}

// Two rows: color tiles on top, depth tiles below (one column per camera).
cv::Mat mosaic(std::vector<CamRx>& cams) {
    std::vector<cv::Mat> color_row, depth_row;
    for (auto& cam : cams) {
        char cl[80], dl[80];
        std::snprintf(cl, sizeof cl, "%s  color %d", cam.name.c_str(), cam.color_fps);
        std::snprintf(dl, sizeof dl, "%s  depth %d", cam.name.c_str(), cam.depth_fps);

        cv::Mat color, depth;
        if (auto c = cam.rx->color().GetData(); c && !c->empty())
            color = cv::Mat(c->height, c->width, CV_8UC3,
                            const_cast<uint8_t*>(c->data.data())).clone();
        if (auto d = cam.rx->depth().GetData(); d && !d->data.empty())
            depth = depth_tile(*d);

        color_row.push_back(labeled_tile(color, cl));
        depth_row.push_back(labeled_tile(depth, dl));
    }
    cv::Mat top, bot, out;
    cv::hconcat(color_row, top);
    cv::hconcat(depth_row, bot);
    cv::vconcat(std::vector<cv::Mat>{top, bot}, out);
    return out;
}
}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";

    Config::instance().load(config_path);
    const auto& root = Config::instance().root();
    const auto unitree      = root["unitree"];
    const int domain_id     = unitree["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;
    const std::string iface;  // empty on purpose — the NIC comes from the DDS config XML
    const std::string only  = (argc >= 3) ? argv[2] : "";   // view just this camera

    const bool has_disp = [] { const char* d = std::getenv("DISPLAY"); return d && d[0]; }();

    std::vector<CamRx> cams;
    for (const auto& name : camera_names_from_yaml(root)) {
        if (!only.empty() && name != only) continue;   // view just this camera
        auto rx = std::make_unique<RealsenseReceiver>();
        if (!rx->start(domain_id, iface, name)) {
            std::fprintf(stderr, "[test_realsense_receiver_viewer] '%s' failed — skipped\n",
                         name.c_str());
            continue;
        }
        cams.push_back(CamRx{name, std::move(rx)});
    }
    if (cams.empty()) {
        std::fprintf(stderr, "[test_realsense_receiver_viewer] no cameras started\n");
        return 1;
    }

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    std::printf("[test_realsense_receiver_viewer] %zu camera(s) on domain=%d iface=%s — %s\n",
                cams.size(), domain_id, iface.c_str(),
                has_disp ? "window (ESC to quit)" : "headless -> /tmp/realsense_view.png");

    auto window = std::chrono::steady_clock::now();
    while (!g_stop) {
        for (auto& cam : cams) {
            auto c = cam.rx->color().GetData();
            auto d = cam.rx->depth().GetData();
            if (c && c->stamp_ns != cam.last_c) { cam.last_c = c->stamp_ns; ++cam.color_n; }
            if (d && d->stamp_ns != cam.last_d) { cam.last_d = d->stamp_ns; ++cam.depth_n; }
        }

        if (has_disp) {
            cv::imshow("realsense rx (multi-cam): color / depth", mosaic(cams));
            if (cv::waitKey(1) == 27) break;   // ESC
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            for (auto& cam : cams) {
                cam.color_fps = cam.color_n; cam.depth_fps = cam.depth_n;
                cam.color_n = cam.depth_n = 0;
                std::printf("  %-12s color %2d fps  depth %2d fps\n",
                            cam.name.c_str(), cam.color_fps, cam.depth_fps);
            }
            if (!has_disp)
                cv::imwrite("/tmp/realsense_view.png", mosaic(cams));
        }
    }

    for (auto& cam : cams) cam.rx->stop();
    return 0;
}
