#pragma once

#include "realsense/transmitter/camera_capture.hpp"   // CameraCaptureConfig
#include "realsense/transmitter/h264_encoder.hpp"      // H264EncoderConfig

#include <string>
#include <vector>

namespace YAML { class Node; }

namespace kist {

// A fully-resolved camera: name + capture + encoder config, with the shared
// `realsense_defaults` merged and the per-camera overrides applied. Produced by
// cameras_from_yaml so the config schema lives in one place, not in each runner.
struct CameraSpec {
    std::string         name;
    CameraCaptureConfig capture;
    H264EncoderConfig   encoder;
};

// One spec per enabled entry in the `realsense_cameras` list — each field falls
// back (camera entry -> realsense_defaults -> struct default). Entries missing a
// name are skipped. Pass Config::instance().root().
std::vector<CameraSpec> cameras_from_yaml(const YAML::Node& root);

// Just the enabled camera names — the Rx side only needs these to subscribe.
std::vector<std::string> camera_names_from_yaml(const YAML::Node& root);

} // namespace kist
