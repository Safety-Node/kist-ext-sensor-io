// Audible variant of the Rx runner — the mic twin of
// test_realsense_receiver_viewer: subscribes to ONE mic and plays its
// chunks on an ALSA output as they arrive, plus a chunk/s line.
//   ./test_mic_receiver_listener [mic_name] [alsa_out] [config]
// mic_name defaults to the first enabled `mics` entry; alsa_out defaults
// to "default" (the plug layer converts rate/channels for the output) —
// find the audible device with `aplay -l` + `speaker-test -D plughw:N,M`.
// Ctrl-C / SIGTERM to stop.

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "mic/mic_config.hpp"
#include "system/mic_receiver.hpp"

#include <alsa/asoundlib.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <deque>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

static std::atomic<bool> g_stop{false};

namespace {

// Chunk hand-off: the DDS hook must stay cheap, and ALSA writes block, so
// chunks queue here and the playback thread drains. Bounded latest-wins —
// if playback stalls, old audio drops (this is a live monitor, not a
// recorder).
class ChunkQueue {
public:
    void push(kist::AudioChunk c) {
        std::lock_guard<std::mutex> lk(m_);
        if (q_.size() >= 10) q_.pop_front();
        q_.push_back(std::move(c));
        cv_.notify_one();
    }
    bool pop(kist::AudioChunk& out) {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, std::chrono::milliseconds(100), [this] { return !q_.empty(); });
        if (q_.empty()) return false;
        out = std::move(q_.front());
        q_.pop_front();
        return true;
    }

private:
    std::deque<kist::AudioChunk> q_;
    std::mutex m_;
    std::condition_variable cv_;
};

} // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string mic_arg     = (argc >= 2) ? argv[1] : "";
    const std::string alsa_out    = (argc >= 3) ? argv[2] : "default";
    const std::string config_path = (argc >= 4) ? argv[3] : "config/config.yaml";

    kist::Config::instance().load(config_path);
    const auto& root = kist::Config::instance().root();

    const auto unitree  = root["unitree"];
    const int domain_id = unitree ? unitree["domain_id"].as<int>(0) : 0;
    if (!kist::apply_dds_config(root)) return 1;
    const std::string iface;  // empty on purpose — the NIC comes from the DDS config XML

    const auto names = kist::mic_names_from_yaml(root);
    std::string name = mic_arg;
    if (name.empty()) {
        if (names.empty()) {
            std::cerr << "[test_mic_receiver_listener] no enabled 'mics' entries in "
                      << config_path << "\n";
            return 1;
        }
        name = names.front();
    }

    ChunkQueue queue;
    std::atomic<uint64_t> received{0};

    kist::MicReceiver rx;
    rx.set_on_chunk([&](const kist::AudioChunk& c) {
        received.fetch_add(1, std::memory_order_relaxed);
        queue.push(c);
    });
    if (!rx.start(domain_id, iface, name)) return 1;

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    std::printf("[test_mic_receiver_listener] mic '%s' -> %s\n", name.c_str(), alsa_out.c_str());

    // Playback thread: the output opens on the first chunk (its rate/
    // channels aren't known until then — chunks are self-describing).
    std::thread player([&] {
        snd_pcm_t* out = nullptr;
        kist::AudioChunk c;
        while (!g_stop) {
            if (!queue.pop(c)) continue;
            if (c.format != "S16_LE") {
                std::cerr << "[test_mic_receiver_listener] unsupported format '"
                          << c.format << "'\n";
                break;
            }
            if (!out) {
                int err = snd_pcm_open(&out, alsa_out.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
                if (err >= 0)
                    err = snd_pcm_set_params(out, SND_PCM_FORMAT_S16_LE,
                                             SND_PCM_ACCESS_RW_INTERLEAVED,
                                             c.channels, c.sample_rate,
                                             /*soft_resample=*/1, /*latency_us=*/300000);
                if (err < 0) {
                    std::cerr << "[test_mic_receiver_listener] cannot open output \""
                              << alsa_out << "\": " << snd_strerror(err)
                              << " (try `aplay -L` for device names)\n";
                    if (out) { snd_pcm_close(out); out = nullptr; }
                    g_stop = true;
                    break;
                }
                std::printf("[test_mic_receiver_listener] playing %uHz x%uch S16_LE\n",
                            c.sample_rate, c.channels);
            }
            const size_t frame_bytes = size_t(c.channels) * 2;
            const auto frames = snd_pcm_uframes_t(c.data.size() / frame_bytes);
            const snd_pcm_sframes_t n = snd_pcm_writei(out, c.data.data(), frames);
            if (n == -EPIPE)  // underrun: queue ran dry (chunk loss / stall)
                snd_pcm_prepare(out);
            else if (n < 0) {
                std::cerr << "[test_mic_receiver_listener] playback error: "
                          << snd_strerror(int(n)) << "\n";
                break;
            }
        }
        if (out) snd_pcm_close(out);
    });

    uint64_t last = 0;
    auto window = std::chrono::steady_clock::now();
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const auto now = std::chrono::steady_clock::now();
        if (now - window < std::chrono::seconds(1)) continue;
        window = now;
        const uint64_t r = received.load(std::memory_order_relaxed);
        std::printf("  %-12s %llu chunk/s\n", name.c_str(),
                    (unsigned long long)(r - last));
        last = r;
    }

    player.join();
    rx.stop();
    return 0;
}
