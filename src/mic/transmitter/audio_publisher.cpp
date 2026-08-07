#include "mic/transmitter/audio_publisher.hpp"

#include "kist_audio_frames.hpp"  // idlc-generated

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <alsa/asoundlib.h>

#include <chrono>
#include <cstring>
#include <iostream>

namespace kist {

namespace {

// Card name fragment -> "hw:N,0". Literal PCM names ("hw:...", "plughw:...",
// "default") pass through — card numbers move between boots, so configs
// should carry names ("L16K6Ch", "UNO") and let this resolve them.
std::string resolve_device(const std::string& device) {
    if (device.rfind("hw:", 0) == 0 || device.rfind("plughw:", 0) == 0 ||
        device == "default")
        return device;
    for (int card = -1; snd_card_next(&card) == 0 && card >= 0;) {
        char* name = nullptr;
        char* longname = nullptr;
        const bool hit =
            (snd_card_get_name(card, &name) == 0 && name &&
             std::string(name).find(device) != std::string::npos) ||
            (snd_card_get_longname(card, &longname) == 0 && longname &&
             std::string(longname).find(device) != std::string::npos);
        free(name);
        free(longname);
        if (hit) return "hw:" + std::to_string(card) + ",0";
    }
    return {};
}

int64_t now_epoch_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

} // namespace

AudioPublisher::AudioPublisher() = default;
AudioPublisher::~AudioPublisher() { stop(); }

bool AudioPublisher::start(int domain_id, const std::string& network_interface,
                           const MicCaptureConfig& cfg, const std::string& topic) {
    if (running_) return true;
    cfg_ = cfg;

    const std::string pcm_name = resolve_device(cfg.device);
    if (pcm_name.empty()) {
        std::cerr << "[AudioPublisher] no ALSA card matching \"" << cfg.device
                  << "\" (see `arecord -l`)\n";
        return false;
    }

    int err = snd_pcm_open(&pcm_, pcm_name.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) {
        std::cerr << "[AudioPublisher] cannot open " << pcm_name << ": "
                  << snd_strerror(err) << "\n";
        pcm_ = nullptr;
        return false;
    }
    // Explicit hw_params, the arecord way — snd_pcm_set_params' own
    // period/buffer choice makes some USB devices (NEVA UNO) fail the
    // first read with -EIO. Exact rate/channels, no resampling: a mode
    // the hardware can't do fails loudly here.
    {
        snd_pcm_hw_params_t* hw = nullptr;
        snd_pcm_hw_params_alloca(&hw);
        snd_pcm_hw_params_any(pcm_, hw);
        if ((err = snd_pcm_hw_params_set_access(pcm_, hw, SND_PCM_ACCESS_RW_INTERLEAVED)) >= 0 &&
            (err = snd_pcm_hw_params_set_format(pcm_, hw, SND_PCM_FORMAT_S16_LE)) >= 0 &&
            (err = snd_pcm_hw_params_set_channels(pcm_, hw, unsigned(cfg.channels))) >= 0 &&
            (err = snd_pcm_hw_params_set_rate(pcm_, hw, unsigned(cfg.sample_rate), 0)) >= 0)
            err = snd_pcm_hw_params(pcm_, hw);
        if (err >= 0)
            err = snd_pcm_prepare(pcm_);
        if (err < 0) {
            std::cerr << "[AudioPublisher] " << pcm_name << " rejects S16_LE "
                      << cfg.sample_rate << "Hz x" << cfg.channels << "ch: "
                      << snd_strerror(err) << " (see `arecord --dump-hw-params`)\n";
            snd_pcm_close(pcm_);
            pcm_ = nullptr;
            return false;
        }
    }

    try {
        // Safe when the embedding process already initialized the factory.
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
        pub_.reset(new Pub(topic));
        pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[AudioPublisher] DDS init failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        snd_pcm_close(pcm_);
        pcm_ = nullptr;
        return false;
    }

    std::cout << "[AudioPublisher] " << pcm_name << " (" << cfg.device << ") "
              << cfg.sample_rate << "Hz x" << cfg.channels << "ch S16_LE, "
              << cfg.chunk_ms << "ms chunks -> " << topic << "\n";
    running_ = true;
    thread_  = std::thread(&AudioPublisher::run, this);
    return true;
}

void AudioPublisher::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
    if (pcm_) {
        snd_pcm_close(pcm_);
        pcm_ = nullptr;
    }
    pub_.reset();
}

void AudioPublisher::run() {
    const snd_pcm_uframes_t chunk_frames =
        snd_pcm_uframes_t(int64_t(cfg_.sample_rate) * cfg_.chunk_ms / 1000);
    const size_t frame_bytes = size_t(cfg_.channels) * 2;  // S16_LE

    kist_msgs::AudioChunk msg;
    msg.sample_rate() = uint32_t(cfg_.sample_rate);
    msg.channels()    = uint32_t(cfg_.channels);
    msg.format()      = "S16_LE";
    msg.frame_id()    = cfg_.frame_id;
    msg.data().resize(chunk_frames * frame_bytes);

    uint64_t seq = 0;
    while (running_) {
        auto* dst = msg.data().data();
        snd_pcm_uframes_t got = 0;
        while (got < chunk_frames && running_) {
            const snd_pcm_sframes_t n =
                snd_pcm_readi(pcm_, dst + got * frame_bytes, chunk_frames - got);
            if (n == -EPIPE) {  // overrun: kernel ring wrapped while we lagged
                overruns_.fetch_add(1, std::memory_order_relaxed);
                snd_pcm_prepare(pcm_);
                continue;
            }
            if (n < 0) {
                std::cerr << "[AudioPublisher] read error: "
                          << snd_strerror(int(n)) << " — capture stopped\n";
                return;
            }
            got += snd_pcm_uframes_t(n);
        }
        if (!running_) break;

        msg.seq()      = seq++;
        msg.stamp_ns() = now_epoch_ns();
        pub_->Write(msg);
        published_.fetch_add(1, std::memory_order_relaxed);
    }
}

} // namespace kist
