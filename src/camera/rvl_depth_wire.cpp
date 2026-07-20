#include "camera/rvl_depth_wire.hpp"

#include "camera/wire_blob.hpp"

namespace kist {

std::vector<uint8_t> serialize_depth(const RvlDepthFrame& frame) {
    BlobWriter w;
    w.u32(uint32_t(frame.width));
    w.u32(uint32_t(frame.height));
    w.u64(frame.sequence);
    w.i64(frame.stamp_ns);
    w.f32(frame.depth_scale);
    w.str(frame.frame_id);
    w.bytes(frame.data);  // RVL bitstream
    return w.take();
}

RvlDepthFrame deserialize_depth(const std::vector<uint8_t>& blob) {
    BlobReader r(blob);
    RvlDepthFrame frame;
    frame.width       = int(r.u32());
    frame.height      = int(r.u32());
    frame.sequence    = r.u64();
    frame.stamp_ns    = r.i64();
    frame.depth_scale = r.f32();
    frame.frame_id    = r.str();
    frame.data        = r.bytes();
    return frame;
}

} // namespace kist
