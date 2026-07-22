// Runs the RealsenseTransmitter as a standalone process — reads config,
// brings up capture + encode-publishers, and streams typed DDS frames until
// stopped. Pair with test_realsense_receiver (or any embedded RealsenseReceiver)
// to check the transport end to end.
//   ./test_realsense_transmitter [config_path]   (default config/config.yaml)
// Ctrl-C / SIGTERM to stop.

#include "system/realsense_transmitter.hpp"
#include "common/config.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";

    kist::Config::instance().load(config_path);
    const auto& root = kist::Config::instance().root();

    const auto unitree      = root["unitree"];
    const int domain_id     = unitree["domain_id"].as<int>(0);
    const std::string iface = unitree["network_interface"].as<std::string>("lo");

    const auto cam = root["camera"];
    if (!cam || !cam["enabled"].as<bool>(true)) {
        std::cerr << "[test_realsense_transmitter] no enabled 'camera' section in "
                  << config_path << "\n";
        return 1;
    }

    kist::CameraCaptureConfig ccfg;
    ccfg.depth_width    = cam["depth_width"].as<int>(ccfg.depth_width);
    ccfg.depth_height   = cam["depth_height"].as<int>(ccfg.depth_height);
    ccfg.depth_fps      = cam["depth_fps"].as<int>(ccfg.depth_fps);
    ccfg.color_enabled  = true;
    ccfg.color_width    = cam["color_width"].as<int>(ccfg.color_width);
    ccfg.color_height   = cam["color_height"].as<int>(ccfg.color_height);
    ccfg.color_fps      = cam["color_fps"].as<int>(ccfg.color_fps);
    ccfg.align_to_color = cam["align_to_color"].as<bool>(ccfg.align_to_color);
    ccfg.depth_frame_id = cam["depth_frame_id"].as<std::string>(ccfg.depth_frame_id);
    ccfg.color_frame_id = cam["color_frame_id"].as<std::string>(ccfg.color_frame_id);
    if (const auto sf = cam["spatial_filter"]) {
        ccfg.spatial_filter.enabled      = sf["enabled"].as<bool>(ccfg.spatial_filter.enabled);
        ccfg.spatial_filter.magnitude    = sf["magnitude"].as<int>(ccfg.spatial_filter.magnitude);
        ccfg.spatial_filter.smooth_alpha = sf["smooth_alpha"].as<float>(ccfg.spatial_filter.smooth_alpha);
        ccfg.spatial_filter.smooth_delta = sf["smooth_delta"].as<float>(ccfg.spatial_filter.smooth_delta);
    }

    kist::H264EncoderConfig ecfg;
    ecfg.width             = ccfg.color_width;
    ecfg.height            = ccfg.color_height;
    ecfg.fps               = ccfg.color_fps;
    ecfg.bitrate_kbps      = cam["color_bitrate_kbps"].as<int>(ecfg.bitrate_kbps);
    ecfg.keyframe_interval = cam["color_keyframe_interval"].as<int>(ecfg.keyframe_interval);
    ecfg.preset            = cam["color_preset"].as<std::string>(ecfg.preset);
    ecfg.tune              = cam["color_tune"].as<std::string>(ecfg.tune);
    ecfg.profile           = cam["color_profile"].as<std::string>(ecfg.profile);

    kist::RealsenseTransmitter tx;
    if (!tx.start(domain_id, iface, ccfg, ecfg))
        return 1;

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    std::printf("[test_realsense_transmitter] publishing on domain=%d iface=%s (%dx%d@%d)\n",
                domain_id, iface.c_str(), ccfg.color_width, ccfg.color_height, ccfg.color_fps);

    while (!g_stop)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    tx.stop();
    return 0;
}
