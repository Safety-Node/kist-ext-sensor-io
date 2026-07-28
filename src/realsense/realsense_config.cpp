#include "realsense/realsense_config.hpp"

#include <yaml-cpp/yaml.h>

namespace kist {

namespace {
// Value lookup with a fallback chain: per-camera entry -> camera_defaults ->
// hardcoded struct default.
template <typename T>
T pick(const YAML::Node& cam, const YAML::Node& def, const char* key, const T& fallback) {
    if (cam[key]) return cam[key].as<T>();
    if (def && def[key]) return def[key].as<T>();
    return fallback;
}
}  // namespace

std::vector<CameraSpec> cameras_from_yaml(const YAML::Node& root) {
    std::vector<CameraSpec> specs;
    const YAML::Node def     = root["realsense_defaults"];
    const YAML::Node cameras = root["realsense_cameras"];
    if (!cameras || !cameras.IsSequence()) return specs;

    for (const auto& cam : cameras) {
        if (!cam["enabled"].as<bool>(true)) continue;
        const std::string name = cam["name"].as<std::string>("");
        if (name.empty()) continue;

        CameraSpec s;
        s.name = name;
        CameraCaptureConfig& ccfg = s.capture;
        H264EncoderConfig&   ecfg = s.encoder;

        ccfg.serial         = cam["serial"].as<std::string>("");   // always per-camera
        ccfg.depth_width    = pick(cam, def, "depth_width",    ccfg.depth_width);
        ccfg.depth_height   = pick(cam, def, "depth_height",   ccfg.depth_height);
        ccfg.depth_fps      = pick(cam, def, "depth_fps",      ccfg.depth_fps);
        ccfg.color_enabled  = true;
        ccfg.color_width    = pick(cam, def, "color_width",    ccfg.color_width);
        ccfg.color_height   = pick(cam, def, "color_height",   ccfg.color_height);
        ccfg.color_fps      = pick(cam, def, "color_fps",      ccfg.color_fps);
        ccfg.align_to_color = pick(cam, def, "align_to_color", ccfg.align_to_color);
        // frame_ids default to the camera name (for per-camera extrinsics later).
        ccfg.depth_frame_id = pick<std::string>(cam, def, "depth_frame_id", name + "_depth");
        ccfg.color_frame_id = pick<std::string>(cam, def, "color_frame_id", name + "_color");
        // spatial_filter: the camera's block wins, else the shared default's.
        if (const YAML::Node sf = cam["spatial_filter"] ? cam["spatial_filter"]
                                                        : (def ? def["spatial_filter"] : YAML::Node())) {
            ccfg.spatial_filter.enabled      = sf["enabled"].as<bool>(ccfg.spatial_filter.enabled);
            ccfg.spatial_filter.magnitude    = sf["magnitude"].as<int>(ccfg.spatial_filter.magnitude);
            ccfg.spatial_filter.smooth_alpha = sf["smooth_alpha"].as<float>(ccfg.spatial_filter.smooth_alpha);
            ccfg.spatial_filter.smooth_delta = sf["smooth_delta"].as<float>(ccfg.spatial_filter.smooth_delta);
        }

        ecfg.width             = ccfg.color_width;
        ecfg.height            = ccfg.color_height;
        ecfg.fps               = ccfg.color_fps;
        ecfg.bitrate_kbps      = pick(cam, def, "color_bitrate_kbps",      ecfg.bitrate_kbps);
        ecfg.keyframe_interval = pick(cam, def, "color_keyframe_interval", ecfg.keyframe_interval);
        ecfg.preset            = pick<std::string>(cam, def, "color_preset",  ecfg.preset);
        ecfg.tune              = pick<std::string>(cam, def, "color_tune",    ecfg.tune);
        ecfg.profile           = pick<std::string>(cam, def, "color_profile", ecfg.profile);

        specs.push_back(std::move(s));
    }
    return specs;
}

std::vector<std::string> camera_names_from_yaml(const YAML::Node& root) {
    std::vector<std::string> names;
    const YAML::Node cameras = root["realsense_cameras"];
    if (!cameras || !cameras.IsSequence()) return names;
    for (const auto& cam : cameras) {
        if (!cam["enabled"].as<bool>(true)) continue;
        if (const std::string name = cam["name"].as<std::string>(""); !name.empty())
            names.push_back(name);
    }
    return names;
}

} // namespace kist
