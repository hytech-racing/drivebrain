#pragma once

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <string>

namespace core {

/**
 * Topic used for logging numbers and booleans without a dedicated protobuf message.
 * Every message on it is a JSON object whose properties are the fields listed under
 * "debug_log_fields" in the drivebrain config.
 */
inline constexpr const char *DEBUG_LOG_FIELDS_TOPIC = "DebugLogFields";

/**
 * Builds the JSON schema for DEBUG_LOG_FIELDS_TOPIC from the "debug_log_fields"
 * section of the drivebrain config.
 *
 * @param config the parsed drivebrain config
 * @return a JSON schema declaring every configured field with its type
 */
inline nlohmann::json debug_log_fields_schema(const nlohmann::json &config) {
    nlohmann::json properties = nlohmann::json::object();

    const auto config_fields = config.find("debug_log_fields");
    if (config_fields == config.end()) {
        return {{"type", "object"}, {"properties", properties}};
    }
    if (!config_fields->is_object()) {
        spdlog::error("debug_log_fields must be an object");
        return {{"type", "object"}, {"properties", properties}};
    }

    const auto add_fields = [&properties](const nlohmann::json &fields, const char *list_key, const char *json_type) {
        const auto names = fields.find(list_key);
        if (names == fields.end()) {
            return;
        }
        if (!names->is_array()) {
            spdlog::error("debug_log_fields/{} must be an array", list_key);
            return;
        }
        for (const auto &name : *names) {
            if (!name.is_string()) {
                spdlog::error("debug_log_fields/{} contains a non-string entry", list_key);
                continue;
            }
            properties[name.get<std::string>()] = {{"type", json_type}};
        }
    };

    add_fields(*config_fields, "numeric", "number");
    add_fields(*config_fields, "boolean", "boolean");

    return {{"type", "object"}, {"properties", properties}};
}

} // namespace core
