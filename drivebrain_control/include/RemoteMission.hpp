#pragma once

#include <RemoteSafety.hpp>

namespace control {
// Internal contract only. No CAN encoding, solenoid ownership, or inferred
// permission from EBS_DATA / the unconfirmed 0x22A layout belongs here.
class RemoteMission {
public:
    enum class State { DISARMED, ARMED, AWAITING_PERMISSION, RUNNING,
                       AWAITING_FINISHED_ACK, FINISHED, EMERGENCY };
    struct Feedback {
        bool start_permitted = false;
        bool finished_ack = false;
        bool emergency_ack = false;
    };
    struct Requests {
        bool start = false;
        bool emergency = false;
        bool finished = false;
    };

    void arm(const core::RssSnapshot& rss, core::SafetyClock::time_point now) {
        if (_state != State::DISARMED) return;
        _go_edges = rss.go_edges; // A held GO or an edge before arming is consumed.
        _unsafe_events = rss.unsafe_events;
        _state = rss.fresh(now) && rss.safe() ? State::ARMED : State::EMERGENCY;
    }
    // Returns a single profile-start event, after the counterpart grants permission.
    bool update(const core::RssSnapshot& rss, Feedback feedback, core::SafetyClock::time_point now) {
        _emergency_ack = feedback.emergency_ack;
        if (active() && (!rss.fresh(now) || !rss.safe() || rss.unsafe_events != _unsafe_events))
            abort();
        if (_state == State::ARMED && rss.go_edges != _go_edges) {
            _state = State::AWAITING_PERMISSION;
            ++_start_requests;
        }
        _go_edges = rss.go_edges;
        if (_state == State::AWAITING_PERMISSION && feedback.start_permitted) {
            _state = State::RUNNING;
            return true;
        }
        if (_state == State::RUNNING && !feedback.start_permitted) abort();
        if (_state == State::AWAITING_FINISHED_ACK && feedback.finished_ack) _state = State::FINISHED;
        return false;
    }
    void braking_complete() {
        if (_state == State::RUNNING) _state = State::AWAITING_FINISHED_ACK;
    }
    void abort() {
        if (active()) _state = State::EMERGENCY;
    }
    // Explicit operator reset, accepted only while disabled with safe, fresh RSS.
    bool reset(bool enabled, const core::RssSnapshot& rss, core::SafetyClock::time_point now) {
        if (enabled || !rss.fresh(now) || !rss.safe()) return false;
        _state = State::DISARMED;
        _go_edges = rss.go_edges;
        _unsafe_events = rss.unsafe_events;
        _emergency_ack = false;
        return true;
    }
    Requests requests() const {
        return {_state == State::AWAITING_PERMISSION, _state == State::EMERGENCY,
                _state == State::AWAITING_FINISHED_ACK};
    }
    State state() const { return _state; }
    uint64_t start_requests() const { return _start_requests; }
    bool emergency_ack() const { return _emergency_ack; }
    bool active() const {
        return _state == State::ARMED || _state == State::AWAITING_PERMISSION ||
               _state == State::RUNNING || _state == State::AWAITING_FINISHED_ACK;
    }
private:
    State _state = State::DISARMED;
    uint64_t _go_edges = 0;
    uint64_t _unsafe_events = 0;
    uint64_t _start_requests = 0;
    bool _emergency_ack = false;
};
}
