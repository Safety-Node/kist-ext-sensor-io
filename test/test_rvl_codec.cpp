// Deterministic probe of the RVL depth codec (no hardware needed).
//
// RVL is lossless, so the core check is a byte-exact encode->decode
// round-trip across the cases the bitstream must handle: zero runs
// (invalid depth), non-zero runs, large deltas, stride padding, and the
// all-zero / single-pixel edges. Also asserts compression actually
// happens on realistic (smooth) data.

#include "camera/rvl_depth_codec.hpp"

#include <cstdio>
#include <cstring>
#include <vector>

using namespace kist;

static int g_failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%-52s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

// Build a Z16 DepthFrame from row-major uint16 pixels, optional extra
// stride padding (bytes) per row to exercise the stride path.
static DepthFrame make_depth(int w, int h, const std::vector<uint16_t>& px,
                             int pad_bytes = 0) {
    DepthFrame f;
    f.width = w;
    f.height = h;
    f.stride_bytes = w * 2 + pad_bytes;
    f.sequence = 7;
    f.stamp_ns = 123456789;
    f.frame_id = "camera_depth";
    f.data.resize(static_cast<size_t>(f.stride_bytes) * h, 0);
    for (int r = 0; r < h; ++r)
        std::memcpy(f.data.data() + r * f.stride_bytes, px.data() + r * w, w * 2);
    return f;
}

// Compare decoded pixels against the logical (unpadded) source.
static bool pixels_match(const DepthFrame& dec, const std::vector<uint16_t>& src) {
    if (dec.stride_bytes != dec.width * 2) return false;  // decoder emits tight rows
    const auto* p = reinterpret_cast<const uint16_t*>(dec.data.data());
    for (size_t i = 0; i < src.size(); ++i)
        if (p[i] != src[i]) return false;
    return true;
}

int main() {
    RvlDepthEncoder enc;
    RvlDepthDecoder dec;

    auto roundtrip = [&](const char* name, int w, int h,
                         const std::vector<uint16_t>& px, int pad = 0) {
        auto f   = make_depth(w, h, px, pad);
        auto rvl = enc.encode(f);
        auto out = dec.decode(rvl);
        check(name, out.width == w && out.height == h && pixels_match(out, px));
    };

    // mixed zero/non-zero runs with up and down deltas
    roundtrip("mixed runs round-trip",
              4, 2, {0, 0, 1000, 1005, 990, 0, 0, 32000});

    // all invalid (all-zero) — pure zero run
    roundtrip("all-zero frame round-trip", 4, 2, std::vector<uint16_t>(8, 0));

    // all valid, monotonic (max positive deltas)
    roundtrip("monotonic non-zero round-trip",
              4, 1, {100, 200, 300, 400});

    // large swing to exercise multi-nibble VLE + zigzag both signs
    roundtrip("large delta both signs", 3, 1, {65535, 1, 40000});

    // single pixel
    roundtrip("single pixel", 1, 1, {4242});

    // stride padding present (row stride > width*2)
    roundtrip("stride padding handled", 4, 2,
              {0, 500, 500, 0, 0, 0, 1234, 1235}, /*pad=*/8);

    // compression actually shrinks smooth data: a gradient over 320x240
    {
        const int w = 320, h = 240;
        std::vector<uint16_t> px(static_cast<size_t>(w) * h);
        for (int r = 0; r < h; ++r)
            for (int c = 0; c < w; ++c)
                px[r * w + c] = static_cast<uint16_t>(1000 + r + c);  // smooth
        auto f   = make_depth(w, h, px);
        auto rvl = enc.encode(f);
        auto out = dec.decode(rvl);
        check("gradient 320x240 round-trip", pixels_match(out, px));
        check("gradient compresses (< 50% of raw)",
              rvl.data.size() < f.data.size() / 2);
    }

    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
