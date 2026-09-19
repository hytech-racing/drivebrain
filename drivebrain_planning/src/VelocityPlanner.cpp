#include "VelocityPlanner.hpp"

namespace planning {
void VelocityPlanner::tick() {
    auto path = loadPathFromCsv("path.csv");
}

std::vector<core::xy_vec<float>> VelocityPlanner::loadPathFromCsv(
    const std::string& filename
) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Unable to open path CSV: " + filename);
    }

    std::vector<core::xy_vec<float>> path;
    std::string line;
    std::size_t line_number = 0;

    while (std::getline(file, line)) {
        ++line_number;

        if (line.empty()) {
            continue;
        }

        std::stringstream stream(line);
        std::string x_text;
        std::string y_text;

        if (!std::getline(stream, x_text, ',') ||
            !std::getline(stream, y_text)) {
            throw std::runtime_error(
                "Invalid path CSV line " + std::to_string(line_number)
            );
        }

        try {
            path.push_back({
                std::stof(x_text),
                std::stof(y_text)
            });
        } catch (const std::exception&) {
            throw std::runtime_error(
                "Invalid numeric value on CSV line " +
                std::to_string(line_number)
            );
        }
    }

    if (path.size() < 2) {
        throw std::runtime_error(
            "Pure-pursuit path requires at least two points"
        );
    }

    return path;
}
} // namespace planning