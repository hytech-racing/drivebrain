#include "RangeCompression.hpp"

#include <memory>

#include <zstd.h>

namespace comms {

std::unique_ptr<uint8_t[]> compressRangeImage(const uint32_t* ranges, std::size_t width,
                                              std::size_t height, std::size_t& output_size) {
    output_size = 0;
    if (!ranges || width == 0 || height == 0) {
        return nullptr;
    }

    const std::size_t count = width * height;
    const std::size_t byte_count = count * sizeof(uint32_t);
    auto differences = std::make_unique<uint32_t[]>(count);
    uint32_t previous = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const uint32_t value = ranges[i];
        differences[i] = value - previous;
        previous = value;
    }

    const std::size_t capacity = ZSTD_compressBound(byte_count);
    if (ZSTD_isError(capacity)) return nullptr;
    auto compressed = std::make_unique<uint8_t[]>(capacity);
    const std::size_t size = ZSTD_compress(compressed.get(), capacity,
                                           differences.get(), byte_count, 1);
    if (ZSTD_isError(size)) return nullptr;
    output_size = size;
    return compressed;
}

}  // namespace comms
