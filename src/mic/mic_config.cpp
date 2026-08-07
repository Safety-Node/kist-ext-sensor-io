#include "mic/mic_config.hpp"

#include <yaml-cpp/yaml.h>

namespace kist {

namespace {
// Value lookup with a fallback chain: per-mic entry -> mic_defaults ->
// hardcoded struct default.
template <typename T>
T pick(const YAML::Node& mic, const YAML::Node& def, const char* key, const T& fallback) {
    if (mic[key]) return mic[key].as<T>();
    if (def && def[key]) return def[key].as<T>();
    return fallback;
}
}  // namespace

std::vector<MicSpec> mics_from_yaml(const YAML::Node& root) {
    std::vector<MicSpec> specs;
    const YAML::Node def  = root["mic_defaults"];
    const YAML::Node mics = root["mics"];
    if (!mics || !mics.IsSequence()) return specs;

    for (const auto& mic : mics) {
        if (!mic["enabled"].as<bool>(true)) continue;
        const std::string name = mic["name"].as<std::string>("");
        if (name.empty()) continue;

        MicSpec s;
        s.name = name;
        MicCaptureConfig& cfg = s.capture;

        cfg.device      = mic["device"].as<std::string>("");   // always per-mic
        if (cfg.device.empty()) continue;
        cfg.sample_rate = pick(mic, def, "sample_rate", cfg.sample_rate);
        cfg.channels    = pick(mic, def, "channels",    cfg.channels);
        cfg.chunk_ms    = pick(mic, def, "chunk_ms",    cfg.chunk_ms);
        cfg.frame_id    = pick<std::string>(mic, def, "frame_id", name);

        specs.push_back(std::move(s));
    }
    return specs;
}

std::vector<std::string> mic_names_from_yaml(const YAML::Node& root) {
    std::vector<std::string> names;
    for (const auto& s : mics_from_yaml(root))
        names.push_back(s.name);
    return names;
}

} // namespace kist
