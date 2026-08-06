#include "mic/receiver/audio_subscriber.hpp"

#include "kist_audio_frames.hpp"  // idlc-generated

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <chrono>
#include <iostream>

namespace kist {

AudioSubscriber::AudioSubscriber() = default;
AudioSubscriber::~AudioSubscriber() { stop(); }

bool AudioSubscriber::start(int domain_id, const std::string& network_interface,
                            const std::string& topic) {
    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
        sub_.reset(new Sub(topic));
        // Reader queue depth (KEEP_LAST N): ~3s at the default 10 chunks/s —
        // audio consumers tap the hook, so the queue only has to ride out
        // callback stalls, not feed a poller.
        sub_->InitChannel(
            [this](const void* msg) { on_audio_update(msg); }, 30);
    } catch (const std::exception& e) {
        std::cerr << "[AudioSubscriber] DDS init failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }

    stop_watchdog_ = false;
    watchdog_thread_ = std::thread(&AudioSubscriber::watchdog_loop, this);
    std::cout << "[AudioSubscriber] started on domain=" << domain_id
              << " interface=" << network_interface
              << " topic=" << topic << "\n";
    return true;
}

void AudioSubscriber::stop() {
    stop_watchdog_ = true;
    if (watchdog_thread_.joinable())
        watchdog_thread_.join();
    sub_.reset();
}

void AudioSubscriber::set_on_chunk(OnChunkFn fn) {
    on_chunk_ = std::move(fn);
}

void AudioSubscriber::on_audio_update(const void* message) {
    const auto& msg = *static_cast<const kist_msgs::AudioChunk*>(message);

    AudioChunk chunk;
    chunk.sequence    = msg.seq();
    chunk.stamp_ns    = msg.stamp_ns();
    chunk.sample_rate = msg.sample_rate();
    chunk.channels    = msg.channels();
    chunk.format      = msg.format();
    chunk.frame_id    = msg.frame_id();
    chunk.data        = msg.data();

    chunk_buf.SetData(AudioChunk(chunk));
    if (on_chunk_)
        on_chunk_(chunk);
}

// Chunks arrive at ~10Hz (chunk_ms=100); 1s of silence clears the buffer
// so consumers see "no audio". Polled at 10ms like the other watchdogs.
void AudioSubscriber::watchdog_loop() {
    using namespace std::chrono_literals;
    constexpr double stale_ms = 1000.0;

    while (!stop_watchdog_) {
        std::this_thread::sleep_for(10ms);

        auto chunk = chunk_buf.GetDataWithTime();
        if (chunk.HasData() && chunk.GetAgeMs() > stale_ms) {
            std::cerr << "[AudioSubscriber] audio stale — cleared\n";
            chunk_buf.Clear();
        }
    }
}

} // namespace kist
