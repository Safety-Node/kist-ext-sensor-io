// Runs one MicTransmitter per enabled entry in `mics` as a standalone
// process — reads config, opens the ALSA devices, and publishes PCM chunks
// over DDS until stopped. Prints per-mic chunk/s once a second.
//   ./test_mic_transmitter [config_path]      (default config/config.yaml)
// Ctrl-C / SIGTERM to stop.

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "mic/mic_config.hpp"
#include "system/mic_transmitter.hpp"

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

    const auto specs = kist::mics_from_yaml(root);
    if (specs.empty()) {
        std::cerr << "[test_mic_transmitter] no enabled 'mics' entries in "
                  << config_path << "\n";
        return 1;
    }

    struct Mic { std::string name; std::unique_ptr<kist::MicTransmitter> tx; };
    std::vector<Mic> mics;
    for (const auto& spec : specs) {
        auto tx = std::make_unique<kist::MicTransmitter>();
        if (!tx->start(domain_id, iface, spec.name, spec.capture)) {
            std::cerr << "[test_mic_transmitter] mic '" << spec.name
                      << "' failed — skipped\n";
            continue;
        }
        mics.push_back({spec.name, std::move(tx)});
    }
    if (mics.empty()) return 1;

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    std::printf("[test_mic_transmitter] publishing %zu mic(s) on domain=%d\n",
                mics.size(), domain_id);

    std::vector<uint64_t> last(mics.size(), 0);
    auto window = std::chrono::steady_clock::now();
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const auto now = std::chrono::steady_clock::now();
        if (now - window < std::chrono::seconds(1)) continue;
        window = now;
        for (size_t i = 0; i < mics.size(); ++i) {
            const uint64_t p = mics[i].tx->published();
            std::printf("  %-12s %llu chunk/s  overruns %llu\n", mics[i].name.c_str(),
                        (unsigned long long)(p - last[i]),
                        (unsigned long long)mics[i].tx->overruns());
            last[i] = p;
        }
    }

    for (auto& mic : mics)
        mic.tx->stop();
    return 0;
}
