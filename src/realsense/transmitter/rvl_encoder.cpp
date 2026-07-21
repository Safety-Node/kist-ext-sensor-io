#include "realsense/transmitter/rvl_encoder.hpp"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace kist {

namespace {

// 4-bit-nibble variable-length encoding, MSB-first bit packing.
class BitWriter {
public:
    explicit BitWriter(std::vector<uint8_t>& buf) : buf_(buf) {}

    void write_vle(uint32_t value) {
        do {
            uint8_t nibble = static_cast<uint8_t>(value & 0x7u);
            value >>= 3;
            if (value) nibble |= 0x8u;
            write_nibble(nibble);
        } while (value);
    }

    void flush() {
        if (bit_pos_ > 0) {
            buf_.push_back(static_cast<uint8_t>(cur_byte_ << (8 - bit_pos_)));
            cur_byte_ = 0;
            bit_pos_  = 0;
        }
    }

private:
    void write_nibble(uint8_t nibble) {
        for (int i = 3; i >= 0; --i) {
            cur_byte_ = static_cast<uint8_t>((cur_byte_ << 1) | ((nibble >> i) & 1));
            if (++bit_pos_ == 8) {
                buf_.push_back(cur_byte_);
                cur_byte_ = 0;
                bit_pos_  = 0;
            }
        }
    }

    std::vector<uint8_t>& buf_;
    uint8_t cur_byte_ = 0;
    int     bit_pos_  = 0;
};

}  // namespace

RvlDepthFrame RvlEncoder::encode(const DepthFrame& frame) const {
    const int pixel_count = frame.width * frame.height;

    // Flatten pixel array (handle stride padding).
    std::vector<uint16_t> pixels(pixel_count);
    for (int row = 0; row < frame.height; ++row) {
        const auto* src = reinterpret_cast<const uint16_t*>(
            frame.data.data() + row * frame.stride_bytes);
        std::copy(src, src + frame.width, pixels.data() + row * frame.width);
    }

    RvlDepthFrame result;
    result.width       = frame.width;
    result.height      = frame.height;
    result.sequence    = frame.sequence;
    result.stamp_ns    = frame.stamp_ns;
    result.frame_id    = frame.frame_id;
    result.depth_scale = frame.depth_scale;
    result.data.reserve(frame.data.size() / 2);

    BitWriter writer(result.data);
    uint16_t prev = 0;
    int i = 0;

    while (i < pixel_count) {
        // Run of zeros (invalid depth).
        int zero_run = 0;
        while (i < pixel_count && pixels[i] == 0) { ++zero_run; ++i; }
        writer.write_vle(static_cast<uint32_t>(zero_run));

        if (i >= pixel_count) break;

        // Run of non-zeros: count first, then encode as zigzag deltas.
        int j = i;
        while (j < pixel_count && pixels[j] != 0) ++j;
        writer.write_vle(static_cast<uint32_t>(j - i));

        for (; i < j; ++i) {
            const int32_t delta = static_cast<int32_t>(pixels[i]) - static_cast<int32_t>(prev);
            prev = pixels[i];
            const uint32_t encoded = (delta >= 0)
                ? static_cast<uint32_t>(delta * 2)
                : static_cast<uint32_t>(-delta * 2 - 1);
            writer.write_vle(encoded);
        }
    }

    writer.flush();
    return result;
}

} // namespace kist
