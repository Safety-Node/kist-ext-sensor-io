// Camera transmitter — run on the machine with the RealSense (robot):
//   ./test_realsense_tx [config_path]
// CameraTransmitter: capture color+depth -> H.264 / RVL encode -> publish
// rt/kist/camera/{color,depth} over the interface in config/config.yaml.
// Ctrl-C to stop. Pair with test_realsense_rx on the LAN machine.

#include "common/config.hpp"
#include "realsense/transmitter/camera_transmitter.hpp"

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
    const auto& root = Config::instance().root();
    const auto u = root["unitree"];
    const auto cam = root["camera"];

    const int         domain    = u["domain_id"].as<int>();
    const std::string interface = u["network_interface"].as<std::string>();

    CameraCaptureConfig ccfg;
    ccfg.depth_width    = cam["depth_width"].as<int>(640);
    ccfg.depth_height   = cam["depth_height"].as<int>(480);
    ccfg.depth_fps      = cam["depth_fps"].as<int>(30);
    ccfg.color_enabled  = true;
    ccfg.color_width    = cam["color_width"].as<int>(640);
    ccfg.color_height   = cam["color_height"].as<int>(480);
    ccfg.color_fps      = cam["color_fps"].as<int>(30);
    ccfg.align_to_color = cam["align_to_color"].as<bool>(true);

    H264EncoderConfig ecfg;
    ecfg.width             = ccfg.color_width;
    ecfg.height            = ccfg.color_height;
    ecfg.fps               = ccfg.color_fps;
    ecfg.bitrate_kbps      = cam["color_bitrate_kbps"].as<int>(4000);
    ecfg.keyframe_interval = cam["color_keyframe_interval"].as<int>(30);

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    CameraTransmitter tx;
    if (!tx.start(domain, interface, ccfg, ecfg))
        return 1;

    std::printf("publishing rt/kist/camera/{color,depth} on %s — Ctrl-C to stop\n",
                interface.c_str());
    while (!g_stop)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    tx.stop();
    return 0;
}
