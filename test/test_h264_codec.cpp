// Deterministic probe of the H.264 color codec (no hardware). H.264 is
// LOSSY, so this can't be byte-exact like RVL — instead it checks the
// encoder produces data (with a keyframe first), the decoder reconstructs
// the right dimensions, and the decoded image is close to the original
// (mean abs error small) on a smooth synthetic pattern. Also checks the
// wire (de)serialize round-trip of the compressed frame is byte-exact.

#include "realsense/transmitter/h264_color_encoder.hpp"
#include "realsense/receiver/h264_color_decoder.hpp"
#include "realsense/h264_color_wire.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

using namespace kist;

static int g_failures = 0;
static void check(const char* name, bool ok) {
    std::printf("%-52s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

// Smooth BGR gradient — compresses well and survives H.264 with low error.
static ColorFrame make_color(int w, int h, uint64_t seq) {
    ColorFrame f;
    f.width = w; f.height = h; f.stride_bytes = w * 3;
    f.sequence = seq; f.stamp_ns = int64_t(seq) * 33'000'000;
    f.frame_id = "camera_color";
    f.data.resize(static_cast<size_t>(w) * h * 3);
    for (int r = 0; r < h; ++r)
        for (int c = 0; c < w; ++c) {
            uint8_t* p = f.data.data() + (r * w + c) * 3;
            p[0] = uint8_t(c % 256);          // B
            p[1] = uint8_t(r % 256);          // G
            p[2] = uint8_t((r + c) % 256);    // R
        }
    return f;
}

static double mean_abs_error(const ColorFrame& a, const ColorFrame& b) {
    if (a.data.size() != b.data.size() || a.data.empty()) return 1e9;
    double acc = 0;
    for (size_t i = 0; i < a.data.size(); ++i)
        acc += std::abs(int(a.data[i]) - int(b.data[i]));
    return acc / a.data.size();
}

int main() {
    const int w = 640, h = 480;
    H264EncoderConfig cfg;
    cfg.width = w; cfg.height = h;
    H264ColorEncoder enc(cfg);
    H264ColorDecoder dec;

    // First encoded frame must be a keyframe carrying data.
    ColorFrame in0 = make_color(w, h, 0);
    auto enc0 = enc.encode(in0);
    check("encode produced a frame", enc0.has_value() && !enc0->data.empty());
    check("first frame is a keyframe", enc0 && enc0->is_keyframe);
    check("metadata carried on compressed frame",
          enc0 && enc0->width == w && enc0->height == h &&
          enc0->sequence == 0 && enc0->frame_id == "camera_color");

    // Wire round-trip of the compressed frame is byte-exact.
    if (enc0) {
        auto back = deserialize_color(serialize_color(*enc0));
        check("color wire round-trip byte-exact",
              back.width == enc0->width && back.height == enc0->height &&
              back.sequence == enc0->sequence && back.is_keyframe == enc0->is_keyframe &&
              back.frame_id == enc0->frame_id && back.data == enc0->data);
    }

    // Decode the keyframe; check dimensions + visual closeness (lossy).
    bool decoded_ok = false, close_enough = false;
    if (enc0) {
        if (auto out = dec.decode(*enc0)) {
            decoded_ok = (out->width == w && out->height == h &&
                          out->stride_bytes == w * 3);
            const double mae = mean_abs_error(in0, *out);
            close_enough = (mae < 8.0);   // typical H.264 luma/chroma error
            std::printf("  (decoded keyframe mean abs error = %.2f / 255)\n", mae);
        }
    }
    check("keyframe decodes to correct dimensions", decoded_ok);
    check("decoded image close to original (lossy)", close_enough);

    // A few more frames: encoder keeps producing, decoder keeps decoding.
    int enc_ok = 0, dec_ok = 0;
    for (uint64_t s = 1; s <= 5; ++s) {
        auto e = enc.encode(make_color(w, h, s));
        if (e) { ++enc_ok; if (dec.decode(*e)) ++dec_ok; }
    }
    check("streaming encode (5/5)", enc_ok == 5);
    check("streaming decode (5/5)", dec_ok == 5);

    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
