// Deterministic probe of compressed-depth wire serialization (no
// hardware, no DDS). serialize_depth -> deserialize_depth must be a
// byte-exact round-trip of every field, and a truncated blob must throw.

#include "realsense/rvl_depth_wire.hpp"

#include <cstdio>
#include <stdexcept>

using namespace kist;

static int g_failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%-46s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

int main() {
    RvlDepthFrame in;
    in.width = 640;
    in.height = 480;
    in.sequence = 12345;
    in.stamp_ns = 987654321012LL;
    in.depth_scale = 0.0010000000474974513f;  // exact float
    in.frame_id = "camera_depth";
    in.data = {0x00, 0xAB, 0xFF, 0x10, 0x7C, 0x00, 0x00, 0x42};

    const auto blob = serialize_depth(in);
    const auto out  = deserialize_depth(blob);

    check("width/height", out.width == 640 && out.height == 480);
    check("sequence", out.sequence == 12345);
    check("stamp_ns", out.stamp_ns == 987654321012LL);
    check("depth_scale exact", out.depth_scale == in.depth_scale);
    check("frame_id", out.frame_id == "camera_depth");
    check("payload bytes exact", out.data == in.data);

    // empty payload (no RVL bytes) still round-trips
    RvlDepthFrame empty;
    empty.frame_id = "x";
    check("empty payload round-trips",
          deserialize_depth(serialize_depth(empty)).data.empty());

    // truncated blob is rejected, not read out of bounds
    bool threw = false;
    try {
        std::vector<uint8_t> t(blob.begin(), blob.begin() + 4);
        deserialize_depth(t);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    check("truncated blob throws", threw);

    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
