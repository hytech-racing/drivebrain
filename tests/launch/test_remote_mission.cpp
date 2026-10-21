#include <RemoteMission.hpp>
#include <LaunchProfile.hpp>
#include <cstdlib>
#include <iostream>

void check(bool value, const char* message) {
    if (!value) { std::cerr << message << '\n'; std::exit(1); }
}

int main() {
    using Mission = control::RemoteMission;
    using State = Mission::State;
    auto now = core::SafetyClock::time_point{std::chrono::seconds(1)};
    core::RssSnapshot rss{};
    rss.received = rss.stop_not_pressed_1 = rss.stop_not_pressed_2 = rss.correct_mode = true;
    rss.received_at = now;
    Mission mission;
    Mission::Feedback counterpart{};
    rss.go = true;
    ++rss.go_edges;
    mission.arm(rss, now);
    check(!mission.update(rss, counterpart, now) && mission.state() == State::ARMED,
          "GO held before arming must not request start");
    rss.go = false;
    mission.update(rss, counterpart, now);
    rss.go = true;
    ++rss.go_edges;
    check(!mission.update(rss, counterpart, now) && mission.requests().start && mission.start_requests() == 1,
          "Fresh GO creates one pending start request, without drive permission");
    for (int i = 0; i < 10; ++i) mission.update(rss, counterpart, now);
    check(mission.start_requests() == 1, "Held GO never repeats a start request");
    counterpart.start_permitted = true;
    check(mission.update(rss, counterpart, now), "Simulated counterpart grants one profile-start event");
    check(!mission.update(rss, counterpart, now), "Permission held high cannot start twice");

    control::LaunchProfile profile;
    control::LaunchProfile::Config config;
    const std::array<float, 4> stopped{};
    profile.start(config, true, stopped, {{0, 0}});
    check(profile.step(0.004f, stopped, true, false, {{0, 0}}).speed_mps > 0,
          "Permission allows existing driving profile to accelerate");
    profile.step(0.004f, {1, 1, 1, 1}, true, false, {{1, 5}});
    check(profile.phase() == control::LaunchProfile::Phase::BRAKING, "OMS travel enters braking");
    profile.step(0.004f, stopped, true, false, {{0, 10}});
    check(profile.phase() == control::LaunchProfile::Phase::COMPLETE, "Simulated braking completes");
    mission.braking_complete();
    check(mission.requests().finished && mission.state() == State::AWAITING_FINISHED_ACK,
          "Normal completion requests Finished and waits for counterpart acknowledgment");
    mission.update(rss, counterpart, now);
    check(mission.requests().finished, "Completion cannot infer acknowledgment from stopped motion");
    counterpart.finished_ack = true;
    mission.update(rss, counterpart, now);
    check(mission.state() == State::FINISHED && !mission.requests().finished, "Finished acknowledgment completes handshake");
    check(!mission.update(rss, counterpart, now), "Finished mission does not automatically restart");

    for (int fault = 0; fault < 7; ++fault) {
        Mission test;
        auto sample = rss;
        test.arm(sample, now);
        ++sample.go_edges;
        check(test.update(sample, counterpart, now), "Start simulated mission for fault injection");
        if (fault == 0) sample.stop_not_pressed_1 = false;
        if (fault == 1) sample.stop_not_pressed_2 = false;
        if (fault == 2) sample.pre_alarm = true;
        if (fault == 3) sample.correct_mode = false;
        if (fault == 4) ++sample.unsafe_events; // STOP pulse recovered between control ticks.
        auto tick = fault == 5 ? now + core::RssSnapshot::timeout + std::chrono::milliseconds(1) : now;
        if (fault == 6) test.abort();
        test.update(sample, counterpart, tick);
        check(test.state() == State::EMERGENCY && test.requests().emergency, "Every stop condition latches Emergency");
        sample = rss;
        sample.received_at = tick;
        counterpart.emergency_ack = true;
        ++sample.go_edges;
        check(!test.update(sample, counterpart, tick) && test.requests().emergency && test.emergency_ack(),
              "RSS recovery, GO and EBS emergency acknowledgment cannot restart or clear Emergency");
        check(!test.reset(true, sample, tick), "Cannot reset while armed/enabled");
        test.arm(sample, tick);
        check(test.requests().emergency, "Rearming alone cannot clear Emergency");
        check(test.reset(false, sample, tick), "Explicit disabled reset accepts healthy RSS");
        test.arm(sample, tick);
        check(!test.update(sample, counterpart, tick), "Reset and rearm still require fresh GO");
    }
    Mission lost_permission;
    lost_permission.arm(rss, now);
    ++rss.go_edges;
    lost_permission.update(rss, counterpart, now);
    counterpart.start_permitted = false;
    lost_permission.update(rss, counterpart, now);
    check(lost_permission.requests().emergency, "Loss of counterpart permission stops a running mission");
    Mission unavailable;
    unavailable.arm(rss, now);
    ++rss.go_edges;
    unavailable.update(rss, {}, now);
    check(unavailable.requests().start && unavailable.state() != State::RUNNING,
          "Default integration boundary never authorizes physical launch");
    unavailable.update(rss, {}, now + core::RssSnapshot::timeout + std::chrono::milliseconds(1));
    check(unavailable.requests().emergency, "RSS timeout also aborts while awaiting permission");
    std::cout << "Remote mission checks passed\n";
}
