#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace comms::yolo {

struct Letterbox {
    int width, height, left, top;
    float scale;
};

inline Letterbox letterbox(int width, int height, int target_width, int target_height) {
    if (width <= 0 || height <= 0 || target_width <= 0 || target_height <= 0)
        throw std::invalid_argument("Invalid letterbox dimensions");
    const float scale = std::min(static_cast<float>(target_width) / width,
                                 static_cast<float>(target_height) / height);
    const int resized_width = std::clamp(static_cast<int>(std::round(width * scale)), 1, target_width);
    const int resized_height = std::clamp(static_cast<int>(std::round(height * scale)), 1, target_height);
    return {resized_width, resized_height, (target_width - resized_width) / 2,
            (target_height - resized_height) / 2, scale};
}

struct Detection {
    float x1, y1, x2, y2, confidence;
    int class_id;
};

inline std::optional<Detection> decode(const float* row, float threshold, const Letterbox& box,
                                       int width, int height) {
    for (int i = 0; i < 6; ++i) if (!std::isfinite(row[i])) return std::nullopt;
    if (row[4] < threshold || row[4] > 1 || row[5] < 0 || row[5] > 1000000 ||
        row[5] != std::floor(row[5])) return std::nullopt;
    Detection d{
        std::clamp((row[0] - box.left) / box.scale, 0.0f, static_cast<float>(width)),
        std::clamp((row[1] - box.top) / box.scale, 0.0f, static_cast<float>(height)),
        std::clamp((row[2] - box.left) / box.scale, 0.0f, static_cast<float>(width)),
        std::clamp((row[3] - box.top) / box.scale, 0.0f, static_cast<float>(height)),
        row[4], static_cast<int>(row[5])};
    if (d.x2 <= d.x1 || d.y2 <= d.y1) return std::nullopt;
    return d;
}

}
