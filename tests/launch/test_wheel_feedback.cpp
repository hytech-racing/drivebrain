#include <StateTracker.hpp>
#include <LaunchController.hpp>
#include <MCAPLogger.hpp>
#include <ControllerManager.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/io_context.hpp>
#include <atomic>
#include <cstdlib>
#include <iostream>

void check(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
template<class Message>
void receive(float rpm) {
    auto message = std::make_shared<Message>();
    message->set_actual_speed_rpm(rpm);
    core::StateTracker::instance().handle_receive_protobuf_message(message);
}

// Exercise the actual Foxglove parameter callback path over a local WebSocket.
class ParameterClient {
public:
    ParameterClient() : _socket(_io) {
        _connection = core::FoxgloveServer::instance().register_param_callback(
            [this](const auto&) { ++_updates; });
        _socket.next_layer().connect({boost::asio::ip::make_address("127.0.0.1"), 5555});
        _socket.set_option(boost::beast::websocket::stream_base::decorator([](auto& request) {
            request.set(boost::beast::http::field::sec_websocket_protocol, "foxglove.websocket.v1");
        }));
        _socket.handshake("127.0.0.1:5555", "/");
        _socket.text(true);
    }
    ~ParameterClient() {
        boost::system::error_code error;
        _socket.next_layer().close(error);
    }
    void set(const char* name, nlohmann::json value) {
        const auto before = _updates.load();
        const auto request = nlohmann::json{{"op", "setParameters"},
            {"parameters", {{{"name", std::string("launchcontroller/") + name}, {"value", value}}}}}.dump();
        _socket.write(boost::asio::buffer(request));
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (_updates == before && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        check(_updates > before, "Foxglove update must reach controller");
    }
private:
    boost::asio::io_context _io;
    boost::beast::websocket::stream<boost::asio::ip::tcp::socket> _socket;
    std::atomic<int> _updates{0};
    boost::signals2::scoped_connection _connection;
};

class IdleController : public control::Controller<core::ControllerOutput, core::VehicleState> {
public:
    float get_dt_sec() override { return 0.004f; }
    core::ControllerOutput step_controller(const core::VehicleState&) override { return {core::SpeedControlOut{}}; }
};
int main(int argc, char** argv) {
    check(argc == 2, "Config path required");
    core::MCAPLogger::create("/tmp/", mcap::McapWriterOptions(""), argv[1]);
    core::FoxgloveServer::create(argv[1]);
    core::StateTracker::create();
    {
        auto& tracker = core::StateTracker::instance();
        check(!tracker.get_latest_state_and_validity().first.wheel_speeds_valid, "Missing feedback invalid");
        receive<hytech::inv1_dynamics>(101);
        receive<hytech::inv2_dynamics>(202);
        receive<hytech::inv3_dynamics>(303);
        check(!tracker.get_latest_state_and_validity().first.wheel_speeds_valid, "Fourth wheel required");
        receive<hytech::inv4_dynamics>(404);
        auto vcr = std::make_shared<hytech_msgs::VCRData_s>();
        vcr->mutable_status()->set_vehicle_state(hytech_msgs::VehicleState_e::READY_TO_DRIVE);
        tracker.handle_receive_protobuf_message(vcr);
        auto state = tracker.get_latest_state_and_validity().first;
        check(state.current_rpms.FL == 101 && state.current_rpms.FR == 202 &&
              state.current_rpms.RL == 303 && state.current_rpms.RR == 404,
              "Inverter 1/2/3/4 must map to FL/FR/RL/RR");
        check(state.wheel_speeds_valid, "All fresh measurements valid");
        check(state.is_ready_to_drive, "Fresh VCR ready-to-drive state accepted");
        auto controller = std::make_shared<control::LaunchController>();
        check(controller->init(), "Controller config loads");
        const auto preview = std::get<core::SpeedControlOut>(controller->preview_controller(state).out);
        check(preview.desired_rpms.FL == 0 && preview.torque_lim_nm.RR == 0, "Preview must not arm");
        const auto idle = std::get<core::SpeedControlOut>(controller->step_controller(state).out);
        check(idle.desired_rpms.FL == 0 && idle.torque_lim_nm.RR == 0, "Default false must not drive");
        std::this_thread::sleep_for(std::chrono::milliseconds(210));
        receive<hytech::inv1_dynamics>(505);
        check(!tracker.get_latest_state_and_validity().first.wheel_speeds_valid, "Each wheel expires independently");
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        check(!tracker.get_latest_state_and_validity().first.is_ready_to_drive, "Stale VCR permission expires");

        // Synthetic healthy state isolates controller logic from physical hardware.
        state = {};
        state.state_is_valid = state.is_ready_to_drive = state.wheel_speeds_valid = true;
        core::VehicleControllerManager::create({std::make_shared<IdleController>(), controller});
        auto& manager = core::VehicleControllerManager::instance();
        check(manager.init(), "Controller manager initializes");
        ParameterClient client;
        client.set("testing_enabled", true);
        check(manager.swap_active_controller(core::LAUNCH_CONTROLLER_INDEX, state), "Select launch controller");
        auto step = [&] { return std::get<core::SpeedControlOut>(manager.step_active_controller(state).out); };
        check(step().torque_lim_nm.FL == 0, "Selecting controller discards pending launch");
        client.set("testing_enabled", false);
        client.set("testing_enabled", true);
        check(step().torque_lim_nm.FL == 2, "False-to-true edge starts launch");
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
        auto accelerating = step();
        check(accelerating.desired_rpms.FL > 0 && accelerating.desired_rpms.FL == accelerating.desired_rpms.FR &&
              accelerating.desired_rpms.FL == accelerating.desired_rpms.RL && accelerating.desired_rpms.FL == accelerating.desired_rpms.RR,
              "All four wheel targets ramp together");
        check(std::get<core::SpeedControlOut>(controller->preview_controller(state).out).desired_rpms.FL == accelerating.desired_rpms.FL,
              "Active preview preserves current command without advancing");
        state.current_rpms = {1000, 1000, 1000, 1000};
        client.set("brake_requested", true);
        const auto braking = step();
        check(std::abs(braking.desired_rpms.FL - 1000) < 0.01f && braking.torque_lim_nm.FL == 1,
              "Foxglove brake request captures measured speed and braking torque limit");
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
        check(step().desired_rpms.FL < braking.desired_rpms.FL, "Braking ramp decreases speed");
        state.wheel_speeds_valid = false;
        check(step().torque_lim_nm.FL == 0, "Stale wheel feedback removes torque");
        state.wheel_speeds_valid = true;
        state.current_rpms = {};
        client.set("brake_requested", false);
        check(step().torque_lim_nm.FL == 0, "Feedback recovery does not auto-restart");
        client.set("testing_enabled", false);
        client.set("testing_enabled", true);
        check(step().torque_lim_nm.FL == 2, "A fresh edge can restart after recovery");
        client.set("testing_enabled", false);
        check(step().desired_rpms.FL == 0, "Disabling test requests braking");
        client.set("braking_distance_m", 0.0);
        client.set("testing_enabled", true);
        check(step().torque_lim_nm.FL == 0, "Invalid configuration cannot start a run");
        core::VehicleControllerManager::destroy();
    }
    core::StateTracker::destroy();
    core::FoxgloveServer::destroy();
    core::MCAPLogger::destroy();
    std::cout << "Wheel feedback and controller integration checks passed\n";
}
