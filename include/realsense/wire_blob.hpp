#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace kist {

// Little tools to pack/unpack a compressed frame into one byte vector
// carried in a DDS byte field (VoxelMapCompressed_.data). We serialize
// ourselves instead of generating a custom IDL type, so the repo needs
// no idlc / ROS2 build tooling — the carrier stays a plain byte bag.
//
// Fixed little-endian layout, length-prefixed strings/bytes. Pure and
// deterministic, so serialize->deserialize is offline-testable.

class BlobWriter {
public:
    void u32(uint32_t v)      { raw(&v, sizeof(v)); }
    void u64(uint64_t v)      { raw(&v, sizeof(v)); }
    void i64(int64_t v)       { raw(&v, sizeof(v)); }
    void f32(float v)         { raw(&v, sizeof(v)); }
    void str(const std::string& s) {
        u32(uint32_t(s.size()));
        raw(s.data(), s.size());
    }
    void bytes(const std::vector<uint8_t>& b) {
        u32(uint32_t(b.size()));
        raw(b.data(), b.size());
    }
    std::vector<uint8_t> take() { return std::move(buf_); }

private:
    void raw(const void* p, std::size_t n) {
        const auto* c = static_cast<const uint8_t*>(p);
        buf_.insert(buf_.end(), c, c + n);
    }
    std::vector<uint8_t> buf_;
};

class BlobReader {
public:
    BlobReader(const uint8_t* data, std::size_t size) : p_(data), end_(data + size) {}
    explicit BlobReader(const std::vector<uint8_t>& b) : BlobReader(b.data(), b.size()) {}

    uint32_t u32() { uint32_t v; raw(&v, sizeof(v)); return v; }
    uint64_t u64() { uint64_t v; raw(&v, sizeof(v)); return v; }
    int64_t  i64() { int64_t  v; raw(&v, sizeof(v)); return v; }
    float    f32() { float    v; raw(&v, sizeof(v)); return v; }
    std::string str() {
        const uint32_t n = u32();
        need(n);
        std::string s(reinterpret_cast<const char*>(p_), n);
        p_ += n;
        return s;
    }
    std::vector<uint8_t> bytes() {
        const uint32_t n = u32();
        need(n);
        std::vector<uint8_t> b(p_, p_ + n);
        p_ += n;
        return b;
    }

private:
    void need(std::size_t n) const {
        if (std::size_t(end_ - p_) < n)
            throw std::runtime_error("[wire_blob] truncated blob");
    }
    void raw(void* out, std::size_t n) {
        need(n);
        std::memcpy(out, p_, n);
        p_ += n;
    }
    const uint8_t* p_;
    const uint8_t* end_;
};

} // namespace kist
