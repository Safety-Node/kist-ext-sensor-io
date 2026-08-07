#pragma once

#include "common/data_buffer.hpp"
#include "mic/audio_chunk.hpp"
#include "mic/receiver/audio_subscriber.hpp"

#include <string>
#include <utility>

namespace kist {

// Mic Rx assembly (own-no-thread) — the mirror of MicTransmitter in
// system/. Audio has no decode stage, so this owns the single Rx unit
// (AudioSubscriber) and just manages its lifecycle; it exists for a uniform
// embedding API with RealsenseReceiver/UwbReceiver. One instance per mic
// name. Empty buffer = no live audio (1s watchdog on the subscriber).
class MicReceiver {
public:
    bool start(int domain_id, const std::string& network_interface,
               const std::string& name);
    void stop();

    // Latest chunk (read from any thread) — liveness/monitoring only.
    DataBuffer<AudioChunk>& chunk() { return sub_.chunk_buf; }

    // Every-chunk hook (runs on the DDS receive thread — keep cheap). The
    // contiguous-audio path for consumers. Set before start().
    void set_on_chunk(AudioSubscriber::OnChunkFn fn) {
        sub_.set_on_chunk(std::move(fn));
    }

private:
    AudioSubscriber sub_;
    bool            running_ = false;
};

} // namespace kist
