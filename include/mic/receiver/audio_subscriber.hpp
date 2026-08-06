#pragma once

#include "common/data_buffer.hpp"
#include "mic/audio_chunk.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace unitree::robot {
template <typename T> class ChannelSubscriber;
}
namespace kist_msgs {
class AudioChunk;
}

namespace kist {

// Rx half for microphone audio — the audio twin of ColorSubscriber:
// subscribes to the idlc-generated kist_msgs::AudioChunk, maps it to an
// AudioChunk, publishes it to a buffer. The buffer is latest-wins, so a
// consumer that needs EVERY chunk (contiguous audio) must tap the
// set_on_chunk hook instead of polling. Watchdog clears the buffer after 1s.
class AudioSubscriber {
public:
    AudioSubscriber();
    ~AudioSubscriber();

    bool start(int domain_id, const std::string& network_interface,
               const std::string& topic = kMicAudioTopic);
    void stop();

    // Runs inside the DDS receive callback right after chunk_buf is
    // updated — the contiguous-audio path. Must stay cheap; set before
    // start().
    using OnChunkFn = std::function<void(const AudioChunk&)>;
    void set_on_chunk(OnChunkFn fn);

    // Latest chunk (read from any thread) — liveness/monitoring, not a
    // contiguous stream.
    DataBuffer<AudioChunk> chunk_buf;

    // internal: DDS callback
    void on_audio_update(const void* message);

private:
    void watchdog_loop();

    using Sub = unitree::robot::ChannelSubscriber<kist_msgs::AudioChunk>;
    std::unique_ptr<Sub> sub_;

    std::thread       watchdog_thread_;
    std::atomic<bool> stop_watchdog_{false};

    OnChunkFn on_chunk_;
};

} // namespace kist
