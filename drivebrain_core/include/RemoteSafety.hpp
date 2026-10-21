#pragma once

#include <chrono>
#include <cstdint>

namespace core {
using SafetyClock = std::chrono::steady_clock;

struct RssSnapshot {
    SafetyClock::time_point received_at{};
    bool received = false;
    bool go = false; // Existing RSS button_k3_pressed; K2 is observed separately.
    bool k2 = false;
    bool stop_not_pressed_1 = false;
    bool stop_not_pressed_2 = false;
    bool pre_alarm = false;
    bool correct_mode = false;
    int radio_link_quality = 0;
    uint64_t go_edges = 0;
    uint64_t unsafe_events = 0;

    bool safe() const {
        return stop_not_pressed_1 && stop_not_pressed_2 && !pre_alarm && correct_mode;
    }
    bool fresh(SafetyClock::time_point now) const {
        return received && now >= received_at && now - received_at <= timeout;
    }
    // Provisional local supervision limit; this is not an RSS protocol change.
    static constexpr auto timeout = std::chrono::milliseconds(500);
};

struct EbsSnapshot {
    SafetyClock::time_point received_at{};
    bool received = false;
    int air_pressure_1 = 0;
    int air_pressure_2 = 0;
    bool engaged = false;
    bool pressure_ok = false;
    bool supervisor_ok = false;
    bool solenoid_1_on = false;
    bool solenoid_2_on = false;
};
}
