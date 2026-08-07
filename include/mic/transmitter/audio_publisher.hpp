#pragma once

#include "mic/audio_chunk.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

// ALSA capture handle (alsa/asoundlib.h stays in the .cpp).
extern "C" { typedef struct _snd_pcm snd_pcm_t; }

namespace unitree::robot {
template <typename T> class ChannelPublisher;
}
namespace kist_msgs {
class AudioChunk;
}

namespace kist {

// Capture settings for one microphone. `device` is either a literal ALSA
// PCM name ("hw:3,0", "plughw:...") or a card name fragment to match
// ("L16K6Ch", "UNO") — card numbers move between boots, names don't.
struct MicCaptureConfig {
    std::string device;
    int         sample_rate = 16000;
    int         channels    = 1;
    int         chunk_ms    = 100;   // PCM per DDS message
    std::string frame_id;            // stamped on published chunks
};

// Tx thread (capture + publish folded, like the camera encode-publishers):
// blocking ALSA reads pace the loop — every chunk_ms of interleaved S16_LE
// PCM becomes one typed DDS message (kist_msgs::AudioChunk). An overrun
// (-EPIPE) is recovered with snd_pcm_prepare and counted; the missing
// audio shows as a stamp_ns jump, not a stall.
class AudioPublisher {
public:
    AudioPublisher();
    ~AudioPublisher();

    // Opens the ALSA device (S16_LE, cfg rate/channels — no resampling;
    // a mode the hardware can't do fails loudly here) + the DDS channel,
    // then runs the capture-publish worker.
    bool start(int domain_id, const std::string& network_interface,
               const MicCaptureConfig& cfg,
               const std::string& topic = kMicAudioTopic);
    void stop();

    // Chunks written to DDS so far (monotonic) — poll the delta over a
    // window for the publish rate. Overruns should stay 0.
    uint64_t published() const { return published_.load(std::memory_order_relaxed); }
    uint64_t overruns()  const { return overruns_.load(std::memory_order_relaxed); }

private:
    void run();

    using Pub = unitree::robot::ChannelPublisher<kist_msgs::AudioChunk>;

    MicCaptureConfig      cfg_;
    snd_pcm_t*            pcm_ = nullptr;
    std::unique_ptr<Pub>  pub_;
    std::thread           thread_;
    std::atomic<bool>     running_{false};
    std::atomic<uint64_t> published_{0};
    std::atomic<uint64_t> overruns_{0};
};

} // namespace kist
