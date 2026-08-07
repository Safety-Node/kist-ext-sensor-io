#pragma once

#include "mic/audio_chunk.hpp"
#include "mic/transmitter/audio_publisher.hpp"

#include <string>

namespace kist {

// Mic Tx assembly (owns no thread of its own) — manages the single Tx unit
// (AudioPublisher: ALSA capture + DDS publish in one thread); it exists for
// a uniform embedding API with RealsenseTransmitter/UwbTransmitter. One
// instance per microphone: `name` namespaces its DDS topic
// (rt/kist/mic/<name>/audio). Config parsing stays out of here.
class MicTransmitter {
public:
    bool start(int domain_id, const std::string& network_interface,
               const std::string& name, const MicCaptureConfig& cfg);
    void stop();

    // Chunks written to DDS so far (monotonic) — poll the delta over a
    // window for the publish rate. Overruns should stay 0.
    uint64_t published() const { return pub_.published(); }
    uint64_t overruns()  const { return pub_.overruns(); }

private:
    AudioPublisher pub_;
    bool           running_ = false;
};

} // namespace kist
