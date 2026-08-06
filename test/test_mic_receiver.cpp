// Runs one MicReceiver per enabled entry in `mics` as a standalone process —
// subscribes and prints per-mic chunk/s + received audio rate once a second.
// Chunks are counted on the every-chunk hook, so the printed rate is exact.
//   ./test_mic_receiver [config_path]      (default config/config.yaml)
// Ctrl-C / SIGTERM to stop.

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "mic/mic_config.hpp"
#include "system/mic_receiver.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";

    kist::Config::instance().load(config_path);
    const auto& root = kist::Config::instance().root();

    const auto unitree  = root["unitree"];
    const int domain_id = unitree ? unitree["domain_id"].as<int>(0) : 0;
    if (!kist::apply_dds_config(root)) return 1;
    const std::string iface;  // empty on purpose — the NIC comes from the DDS config XML

    const auto names = kist::mic_names_from_yaml(root);
    if (names.empty()) {
        std::cerr << "[test_mic_receiver] no enabled 'mics' entries in "
                  << config_path << "\n";
        return 1;
    }

    struct Mic {
        std::string name;
        std::unique_ptr<kist::MicReceiver> rx;
        std::atomic<uint64_t> chunks{0};
        std::atomic<uint64_t> bytes{0};
    };
    std::vector<std::unique_ptr<Mic>> mics;
    for (const auto& name : names) {
        auto mic = std::make_unique<Mic>();
        mic->name = name;
        mic->rx   = std::make_unique<kist::MicReceiver>();
        auto* m = mic.get();
        m->rx->set_on_chunk([m](const kist::AudioChunk& c) {
            m->chunks.fetch_add(1, std::memory_order_relaxed);
            m->bytes.fetch_add(c.data.size(), std::memory_order_relaxed);
        });
        if (!m->rx->start(domain_id, iface, name)) {
            std::cerr << "[test_mic_receiver] mic '" << name << "' failed — skipped\n";
            continue;
        }
        mics.push_back(std::move(mic));
    }
    if (mics.empty()) return 1;

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    std::printf("[test_mic_receiver] listening to %zu mic(s) on domain=%d\n",
                mics.size(), domain_id);

    std::vector<std::pair<uint64_t, uint64_t>> last(mics.size(), {0, 0});
    auto window = std::chrono::steady_clock::now();
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const auto now = std::chrono::steady_clock::now();
        if (now - window < std::chrono::seconds(1)) continue;
        window = now;
        for (size_t i = 0; i < mics.size(); ++i) {
            const uint64_t c = mics[i]->chunks.load(std::memory_order_relaxed);
            const uint64_t b = mics[i]->bytes.load(std::memory_order_relaxed);
            std::printf("  %-12s %llu chunk/s  %.1f KB/s\n", mics[i]->name.c_str(),
                        (unsigned long long)(c - last[i].first),
                        double(b - last[i].second) / 1e3);
            last[i] = {c, b};
        }
    }

    for (auto& mic : mics)
        mic->rx->stop();
    return 0;
}
