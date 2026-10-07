#include "DataCompression.hpp"

#include <limits>
#include <memory>

#include <zstd.h>

namespace comms {

std::unique_ptr<uint8_t[]> splitBayerImage(const uint8_t* input, std::size_t width, std::size_t height) {
    auto output = std::make_unique<uint8_t[]>(width * height);
    std::size_t offset = 0;

    // Copy each color into its own contiguous plane in RGGB order.
    for (std::size_t y = 0; y < height; y += 2) { // R
        for (std::size_t x = 0; x < width; x += 2) {
            output[offset++] = input[y * width + x];
        }
    }
    for (std::size_t y = 0; y < height; y += 2) { // G1
        for (std::size_t x = 1; x < width; x += 2) {
            output[offset++] = input[y * width + x];
        }
    }
    for (std::size_t y = 1; y < height; y += 2) { // G2
        for (std::size_t x = 0; x < width; x += 2) {
            output[offset++] = input[y * width + x];
        }
    }
    for (std::size_t y = 1; y < height; y += 2) { // B
        for (std::size_t x = 1; x < width; x += 2) {
            output[offset++] = input[y * width + x];
        }
    }
    return output;
}

std::unique_ptr<uint8_t[]> computeDifferences(const uint8_t* input, std::size_t width, std::size_t height) {
    auto output = std::make_unique<uint8_t[]>(width * height);
    output[0] = input[0];
    for (std::size_t i = 1; i < width * height; ++i) {
        output[i] = (input[i] - input[i - 1]) % 256;
    }
    return output;
}

std::unique_ptr<uint8_t[]> applyZstdCompression(const uint8_t* input, std::size_t input_size,
                                                  std::size_t& output_size) {
    output_size = 0;
    if (input == nullptr || input_size == 0) return nullptr;

    // get expected max size for compressed output to size the array
    const auto capacity = ZSTD_compressBound(input_size);
    if (ZSTD_isError(capacity)) return nullptr;
    auto output = std::make_unique<uint8_t[]>(capacity);
    const auto size = ZSTD_compress(output.get(), capacity, input, input_size, 1); // run zstd level 1 compression
    if (ZSTD_isError(size)) return nullptr;
    output_size = size;
    return output;
}

std::unique_ptr<uint8_t[]> compressBayerImage(const uint8_t* input, std::size_t width,
                                               std::size_t height, std::size_t& output_size) {
    output_size = 0;

    auto planes = splitBayerImage(input, width, height);
    if (!planes) return nullptr;

    auto differences = computeDifferences(planes.get(), width, height);
    if (!differences) return nullptr;

    return applyZstdCompression(differences.get(), width * height, output_size);
}

void averageBayer2x2ToBgr(const uint8_t* input, int width, int height, cv::Mat& output) {

    output.create(height / 2, width / 2, CV_8UC3); // don't handle edge cases empty image is still valid
    for (int y = 0; y + 1 < height; y += 2) {
        for (int x = 0; x + 1 < width; x += 2) {
            // start at red pixel and average adjacent values into a BGR entry
            const uint8_t red = input[y * width + x];
            const uint8_t green_top = input[y * width + x + 1];
            const uint8_t green_bottom = input[(y+1) * width + x];
            const uint8_t blue = input[(y+1) * width + x + 1];
            const uint8_t green = static_cast<uint8_t>((green_top + green_bottom) / 2);
            output.at<cv::Vec3b>(y / 2, x / 2) = cv::Vec3b(blue, green, red);
        }
    }
}

} // namespace comms
