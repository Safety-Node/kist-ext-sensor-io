#pragma once

#include "mic/transmitter/audio_publisher.hpp"   // MicCaptureConfig

#include <string>
#include <vector>

namespace YAML { class Node; }

namespace kist {

// A fully-resolved microphone: name + capture config, with the shared
// `mic_defaults` merged and the per-mic overrides applied. Produced by
// mics_from_yaml so the config schema lives in one place, not in each
// runner (same pattern as CameraSpec / cameras_from_yaml).
struct MicSpec {
    std::string      name;
    MicCaptureConfig capture;
};

// One spec per enabled entry in the `mics` list — each field falls back
// (mic entry -> mic_defaults -> struct default). Entries missing a name or
// device are skipped. Pass Config::instance().root().
std::vector<MicSpec> mics_from_yaml(const YAML::Node& root);

// Just the enabled mic names — the Rx side only needs these to subscribe.
std::vector<std::string> mic_names_from_yaml(const YAML::Node& root);

} // namespace kist
