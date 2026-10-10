#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

namespace comms {

/**
 * @brief Difference a row-major uint32 range image modulo 2^32 and compress it with Zstd.
 *        Differences are stored as uint32 values, starting with the first range
 *        value unchanged. Decoding is Zstd decompression followed by cumulative
 *        uint32 addition in row-major order.
 * @param ranges Contiguous row-major range values, including zero-valued returns.
 * @param width Number of columns.
 * @param height Number of rows.
 * @param output_size Receives the number of compressed bytes, or zero on failure.
 * @return Owning pointer to the compressed bytes, or nullptr on failure.
 */
std::unique_ptr<uint8_t[]> compressRangeImage(const uint32_t* ranges, std::size_t width,
                                              std::size_t height, std::size_t& output_size);

}  // namespace comms
