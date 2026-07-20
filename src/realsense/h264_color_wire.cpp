#include "realsense/h264_color_wire.hpp"

#include "realsense/wire_blob.hpp"

namespace kist {

std::vector<uint8_t> serialize_color(const H264ColorFrame& frame) {
    BlobWriter w;
    w.u32(uint32_t(frame.width));
    w.u32(uint32_t(frame.height));
    w.u64(frame.sequence);
    w.i64(frame.stamp_ns);
    w.u32(frame.is_keyframe ? 1u : 0u);
    w.str(frame.frame_id);
    w.bytes(frame.data);  // Annex-B NAL units
    return w.take();
}

H264ColorFrame deserialize_color(const std::vector<uint8_t>& blob) {
    BlobReader r(blob);
    H264ColorFrame frame;
    frame.width       = int(r.u32());
    frame.height      = int(r.u32());
    frame.sequence    = r.u64();
    frame.stamp_ns    = r.i64();
    frame.is_keyframe = (r.u32() != 0);
    frame.frame_id    = r.str();
    frame.data        = r.bytes();
    return frame;
}

} // namespace kist
