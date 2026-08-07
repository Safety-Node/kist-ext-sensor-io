// Visual mic Rx runner — the spectrogram twin of
// test_realsense_receiver_viewer: subscribes to every mic in `mics` (or one
// named via argv[1]) and shows a scrolling STFT spectrogram per mic, one
// row each, labeled with name + chunk/s. Channel 0 only (the DSP output on
// array mics); low frequencies at the bottom, dB-scaled JET colormap.
//   ./test_mic_receiver_stft [mic_name] [config_path]
// With a DISPLAY it opens a window (ESC to quit); headless it writes
// /tmp/mic_stft.png once per second.

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "mic/mic_config.hpp"
#include "system/mic_receiver.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace kist;

static std::atomic<bool> g_stop{false};

namespace {

constexpr int    kFft  = 512;         // STFT window (samples)
constexpr int    kHop  = 256;         // hop (50% overlap)
constexpr int    kBins = kFft / 2;    // displayed frequency bins
constexpr int    kCols = 768;         // time columns (scroll width)
constexpr double kDbFloor = -80.0;    // dB mapped to black
const cv::Scalar kText(230, 230, 230);

struct MicRx {
    std::string name;
    std::unique_ptr<MicReceiver> rx;

    // Hook side: channel-0 samples ([-1,1]) queue up under the lock; the
    // render loop drains them into the STFT.
    std::mutex         m;
    std::vector<float> pending;
    uint32_t           rate = 0, channels = 0;

    std::atomic<uint64_t> chunks{0};
    int chunk_rate = 0, chunk_n = 0;   // latched once/sec for the label

    std::vector<float> tail;                     // carry-over between hops
    cv::Mat spec{kBins, kCols, CV_8UC1, cv::Scalar(0)};
};

// One Hann-windowed FFT column of `spec`: samples[0..kFft) -> dB magnitudes,
// low frequencies at the bottom row.
void stft_column(MicRx& mic, const float* samples) {
    static thread_local std::vector<float> hann;
    if (hann.empty()) {
        hann.resize(kFft);
        for (int i = 0; i < kFft; ++i)
            hann[i] = 0.5f * (1.f - std::cos(2.f * float(CV_PI) * i / (kFft - 1)));
    }
    cv::Mat in(1, kFft, CV_32F);
    for (int i = 0; i < kFft; ++i)
        in.at<float>(0, i) = samples[i] * hann[i];
    cv::Mat out;
    cv::dft(in, out, cv::DFT_COMPLEX_OUTPUT);

    // Scroll left one column, write the new one on the right.
    mic.spec(cv::Rect(1, 0, kCols - 1, kBins)).copyTo(mic.spec(cv::Rect(0, 0, kCols - 1, kBins)));
    for (int b = 0; b < kBins; ++b) {
        const float re = out.at<cv::Vec2f>(0, b)[0];
        const float im = out.at<cv::Vec2f>(0, b)[1];
        const double mag = std::sqrt(double(re) * re + double(im) * im) * 2.0 / kFft;
        const double db  = 20.0 * std::log10(mag + 1e-9);
        const int v = int(std::clamp((db - kDbFloor) / -kDbFloor, 0.0, 1.0) * 255.0);
        mic.spec.at<uint8_t>(kBins - 1 - b, kCols - 1) = uint8_t(v);
    }
}

// Drain pending samples through the STFT (as many hops as they cover).
void process(MicRx& mic) {
    std::vector<float> in;
    {
        std::lock_guard<std::mutex> lk(mic.m);
        in.swap(mic.pending);
    }
    if (in.empty()) return;
    mic.tail.insert(mic.tail.end(), in.begin(), in.end());
    size_t off = 0;
    while (mic.tail.size() - off >= size_t(kFft)) {
        stft_column(mic, mic.tail.data() + off);
        off += kHop;
    }
    mic.tail.erase(mic.tail.begin(), mic.tail.begin() + long(off));
}

// Colormapped tile with a title bar (name, chunk/s, rate/channels).
cv::Mat mic_tile(MicRx& mic) {
    cv::Mat color;
    cv::applyColorMap(mic.spec, color, cv::COLORMAP_JET);
    char label[96];
    if (mic.rate)
        std::snprintf(label, sizeof label, "%s  %d chunk/s  %uHz x%uch  (0..%ukHz)",
                      mic.name.c_str(), mic.chunk_rate, mic.rate, mic.channels,
                      mic.rate / 2000);
    else
        std::snprintf(label, sizeof label, "%s  no signal", mic.name.c_str());
    cv::rectangle(color, {0, 0}, {kCols, 20}, cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(color, label, {6, 15}, cv::FONT_HERSHEY_SIMPLEX, 0.42, kText, 1, cv::LINE_AA);
    return color;
}

cv::Mat mosaic(std::vector<std::unique_ptr<MicRx>>& mics) {
    std::vector<cv::Mat> rows;
    for (auto& mic : mics)
        rows.push_back(mic_tile(*mic));
    cv::Mat out;
    cv::vconcat(rows, out);
    return out;
}

} // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string only        = (argc >= 2) ? argv[1] : "";   // view just this mic
    const std::string config_path = (argc >= 3) ? argv[2] : "config/config.yaml";

    Config::instance().load(config_path);
    const auto& root = Config::instance().root();
    const auto unitree  = root["unitree"];
    const int domain_id = unitree ? unitree["domain_id"].as<int>(0) : 0;
    if (!kist::apply_dds_config(root)) return 1;
    const std::string iface;  // empty on purpose — the NIC comes from the DDS config XML

    const bool has_disp = [] { const char* d = std::getenv("DISPLAY"); return d && d[0]; }();

    std::vector<std::unique_ptr<MicRx>> mics;
    for (const auto& name : mic_names_from_yaml(root)) {
        if (!only.empty() && name != only) continue;
        auto mic = std::make_unique<MicRx>();
        mic->name = name;
        mic->rx   = std::make_unique<MicReceiver>();
        auto* m = mic.get();
        m->rx->set_on_chunk([m](const AudioChunk& c) {
            if (c.format != "S16_LE" || c.channels == 0) return;
            m->chunks.fetch_add(1, std::memory_order_relaxed);
            const auto* s = reinterpret_cast<const int16_t*>(c.data.data());
            const size_t frames = c.data.size() / (size_t(c.channels) * 2);
            std::lock_guard<std::mutex> lk(m->m);
            m->rate     = c.sample_rate;
            m->channels = c.channels;
            for (size_t i = 0; i < frames; ++i)                 // channel 0 only
                m->pending.push_back(float(s[i * c.channels]) / 32768.f);
        });
        if (!m->rx->start(domain_id, iface, name)) {
            std::fprintf(stderr, "[test_mic_receiver_stft] '%s' failed — skipped\n",
                         name.c_str());
            continue;
        }
        mics.push_back(std::move(mic));
    }
    if (mics.empty()) {
        std::fprintf(stderr, "[test_mic_receiver_stft] no mics started\n");
        return 1;
    }

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    std::printf("[test_mic_receiver_stft] %zu mic(s) on domain=%d — %s\n",
                mics.size(), domain_id,
                has_disp ? "window (ESC to quit)" : "headless -> /tmp/mic_stft.png");

    auto window = std::chrono::steady_clock::now();
    while (!g_stop) {
        for (auto& mic : mics)
            process(*mic);

        if (has_disp) {
            cv::imshow("mic rx: STFT spectrogram", mosaic(mics));
            if (cv::waitKey(15) == 27) break;   // ESC
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            for (auto& mic : mics) {
                const auto c = mic->chunks.load(std::memory_order_relaxed);
                mic->chunk_rate = int(c - mic->chunk_n);
                mic->chunk_n    = int(c);
                std::printf("  %-12s %d chunk/s\n", mic->name.c_str(), mic->chunk_rate);
            }
            if (!has_disp)
                cv::imwrite("/tmp/mic_stft.png", mosaic(mics));
        }
    }

    for (auto& mic : mics)
        mic->rx->stop();
    return 0;
}
