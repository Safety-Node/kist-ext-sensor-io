#include "realsense/receiver/depth_decoder.hpp"

#include <pthread.h>

#include <chrono>
#include <cstdint>
#include <vector>

namespace kist {

// ── RvlDepthDecoder (pure Module) ────────────────────────────────────

namespace {

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

// ── DepthDecodeThread ────────────────────────────────────────────────

bool DepthDecodeThread::start(DataBuffer<RvlDepthFrame>& source) {
    if (running_) return true;
    source_ = &source;
    running_ = true;
    thread_ = std::thread(&DepthDecodeThread::run, this);
    return true;
}

void DepthDecodeThread::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
}

void DepthDecodeThread::run() {
    pthread_setname_np(pthread_self(), "rs-depth-dec");
    int64_t last_stamp = -1;
    while (running_) {
        auto frame = source_->GetData();
        if (frame && frame->stamp_ns != last_stamp) {
            last_stamp = frame->stamp_ns;
            out.SetData(decoder_.decode(*frame));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

} // namespace kist
