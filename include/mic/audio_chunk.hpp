#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace kist {

// Audio on the wire. The rt/ prefix keeps it ROS2-tool-visible as
// /kist/mic/audio; the payload is the idlc-generated kist_msgs::AudioChunk
// (interleaved PCM in its .data). Lives with the chunk contract so Tx and
// Rx agree without either including the other's header (same pattern as
// kCameraColorTopic).
inline constexpr const char* kMicAudioTopic = "rt/kist/mic/audio";

// Per-mic audio topic: rt/kist/mic/<name>/audio. Multi-mic setups
// namespace by name; the name is the DDS routing identity linking a Tx
// mic to its Rx consumer.
inline std::string mic_audio_topic(const std::string& name) {
    return "rt/kist/mic/" + name + "/audio";
}

// One chunk of interleaved PCM (chunk_ms worth of frames). Self-describing:
// rate/channels/format ride along, so a consumer can interpret `data`
// without knowing which mic produced it.
struct AudioChunk {
    std::uint64_t sequence    = 0;   // per-mic chunk counter (gap detection)
    std::int64_t  stamp_ns    = 0;   // capture clock at chunk end (epoch ns)
    std::uint32_t sample_rate = 0;   // Hz
    std::uint32_t channels    = 0;
    std::string   format;            // PCM sample format, e.g. "S16_LE"
    std::string   frame_id;
    std::vector<std::uint8_t> data;  // interleaved PCM payload
};

} // namespace kist
