#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

namespace comms {

/**
 * @brief Split an RGGB Bayer image into contiguous R, G1, G2, B planes.
 * @param input Pointer to the Bayer image data.
 * @param width Image width in pixels.
 * @param height Image height in pixels.
 * @return Owning pointer to the planes, or nullptr for invalid input or dimensions.
 */
std::unique_ptr<uint8_t[]> splitBayerImage(const uint8_t* input, std::size_t width, std::size_t height);

/**
 * @brief Difference adjacent samples in the flattened planes above.
 *        The first sample is unchanged. Uses modulo 256 arithmetic to handle wraparound.
 * @param input Pointer to the layered Bayer image data.
 * @param width Image width in pixels.
 * @param height Image height in pixels.
 * @return Owning pointer to the differences, or nullptr for invalid input or dimensions.
 */
std::unique_ptr<uint8_t[]> computeDifferences(const uint8_t* input, std::size_t width, std::size_t height);

/**
 * @brief Compress the given bytes with Zstd level 1.
 * @param input Pointer to the bytes to compress.
 * @param input_size Number of input bytes.
 * @param output_size Receives the number of compressed bytes, or zero on failure.
 * @return Owning pointer to the compressed bytes, or nullptr on failure.
 */
std::unique_ptr<uint8_t[]> applyZstdCompression(const uint8_t* input, std::size_t input_size, std::size_t& output_size);

/**
 * @brief Split an RGGB Bayer image into planes, difference the bytes, and compress with Zstd.
 * @param input Pointer to the Bayer image data.
 * @param width Image width in pixels.
 * @param height Image height in pixels.
 * @param output_size Receives the number of compressed bytes, or zero on failure.
 * @return Owning pointer to the compressed bytes, or nullptr on failure.
 */
std::unique_ptr<uint8_t[]> compressBayerImage(const uint8_t* input, std::size_t width,
                                               std::size_t height, std::size_t& output_size);

} // namespace comms
