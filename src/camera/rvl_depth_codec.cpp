#include "camera/rvl_depth_codec.hpp"

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

class BitReader {
public:
    BitReader(const uint8_t* data, size_t size) : data_(data), size_(size) {}

    uint32_t read_vle() {
        uint32_t value = 0;
        uint32_t shift = 0;
        uint8_t  nibble;
        do {
            nibble = read_nibble();
            value |= static_cast<uint32_t>(nibble & 0x7u) << shift;
            shift += 3;
        } while (nibble & 0x8u);
        return value;
    }

private:
    uint8_t read_nibble() {
        uint8_t nibble = 0;
        for (int i = 3; i >= 0; --i) {
            if (bit_pos_ == 0) {
                cur_byte_ = (byte_pos_ < size_) ? data_[byte_pos_++] : 0;
                bit_pos_  = 8;
            }
            nibble = static_cast<uint8_t>(nibble | (((cur_byte_ >> --bit_pos_) & 1) << i));
        }
        return nibble;
    }

    const uint8_t* data_;
    size_t         size_;
    size_t         byte_pos_ = 0;
    uint8_t        cur_byte_ = 0;
    int            bit_pos_  = 0;
};

}  // namespace

RvlDepthFrame RvlDepthEncoder::encode(const DepthFrame& frame) const {
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

DepthFrame RvlDepthDecoder::decode(const RvlDepthFrame& frame) const {
    const int pixel_count = frame.width * frame.height;

    DepthFrame result;
    result.width        = frame.width;
    result.height       = frame.height;
    result.stride_bytes = frame.width * static_cast<int>(sizeof(uint16_t));
    result.stamp_ns     = frame.stamp_ns;
    result.frame_id     = frame.frame_id;
    result.depth_scale  = frame.depth_scale;
    result.data.resize(static_cast<size_t>(pixel_count) * sizeof(uint16_t), 0);

    auto* pixels = reinterpret_cast<uint16_t*>(result.data.data());

    BitReader reader(frame.data.data(), frame.data.size());
    uint16_t prev = 0;
    int i = 0;

    while (i < pixel_count) {
        const int zero_run = static_cast<int>(reader.read_vle());
        for (int k = 0; k < zero_run && i < pixel_count; ++k, ++i)
            pixels[i] = 0;

        if (i >= pixel_count) break;

        const int nonzero_run = static_cast<int>(reader.read_vle());
        for (int k = 0; k < nonzero_run && i < pixel_count; ++k, ++i) {
            const uint32_t encoded = reader.read_vle();
            const int32_t delta = (encoded & 1u)
                ? -static_cast<int32_t>((encoded + 1u) >> 1)
                :  static_cast<int32_t>(encoded >> 1);
            prev      = static_cast<uint16_t>(static_cast<int32_t>(prev) + delta);
            pixels[i] = prev;
        }
    }

    return result;
}

} // namespace kist
