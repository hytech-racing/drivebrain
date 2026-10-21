#pragma once

#include <RemoteSafety.hpp>
#include <Telemetry.hpp>
#include <google/protobuf/struct.pb.h>

namespace core {
// Called at 10 Hz by the executive, including while manual teleop is selected.
inline void log_remote_safety(const RssSnapshot& rss, const EbsSnapshot& ebs, SafetyClock::time_point now) {
    auto msg = std::make_shared<google::protobuf::Struct>();
    auto& f = *msg->mutable_fields();
    const auto stamp_ms = [](auto stamp) {
        return std::chrono::duration<double, std::milli>(stamp.time_since_epoch()).count();
    };
    const auto age_ms = [now](auto stamp) {
        return std::chrono::duration<double, std::milli>(now - stamp).count();
    };
    f["rss_received"].set_bool_value(rss.received);
    f["rss_received_steady_ms"].set_number_value(stamp_ms(rss.received_at));
    f["rss_age_ms"].set_number_value(rss.received ? age_ms(rss.received_at) : -1);
    f["rss_fresh"].set_bool_value(rss.fresh(now));
    f["go"].set_bool_value(rss.go);
    f["k2"].set_bool_value(rss.k2);
    f["stop_not_pressed_1"].set_bool_value(rss.stop_not_pressed_1);
    f["stop_not_pressed_2"].set_bool_value(rss.stop_not_pressed_2);
    f["pre_alarm"].set_bool_value(rss.pre_alarm);
    f["correct_mode"].set_bool_value(rss.correct_mode);
    f["radio_link_quality"].set_number_value(rss.radio_link_quality);
    f["go_edges"].set_number_value(rss.go_edges);
    f["unsafe_events"].set_number_value(rss.unsafe_events);
    f["ebs_received"].set_bool_value(ebs.received);
    f["ebs_received_steady_ms"].set_number_value(stamp_ms(ebs.received_at));
    f["ebs_age_ms"].set_number_value(ebs.received ? age_ms(ebs.received_at) : -1);
    f["air_pressure_1"].set_number_value(ebs.air_pressure_1);
    f["air_pressure_2"].set_number_value(ebs.air_pressure_2);
    f["ebs_engaged"].set_bool_value(ebs.engaged);
    f["ebs_pressure_ok"].set_bool_value(ebs.pressure_ok);
    f["ebs_supervisor_ok"].set_bool_value(ebs.supervisor_ok);
    f["ebs_solenoid_1_on"].set_bool_value(ebs.solenoid_1_on);
    f["ebs_solenoid_2_on"].set_bool_value(ebs.solenoid_2_on);
    log_mcap_only(msg, "remote_safety/state");
    log_foxglove_only(msg, "remote_safety/state");
}
}
