#include <StateTracker.hpp>
#include <LaunchController.hpp>
#include <MCAPLogger.hpp>
#include <ControllerManager.hpp>
#include <OMSComms.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/io_context.hpp>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <cstring>

void check(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
template<class Message>
void receive(float rpm) {
    auto message = std::make_shared<Message>();
    message->set_actual_speed_rpm(rpm);
    core::StateTracker::instance().handle_receive_protobuf_message(message);
}
void receive_oms(float time, float vx, float vy, float yaw_rate) {
    auto message = std::make_shared<hytech_msgs::OMSData>();
    message->set_sample_time_s(time);
    message->mutable_rate_hor_deg_s()->set_z(yaw_rate);
    message->mutable_motion()->set_vel_x_kmh(vx);
    message->mutable_motion()->set_vel_y_kmh(vy);
    core::StateTracker::instance().handle_receive_protobuf_message(message);
}

void check_oms_tcp_stream() {
    using boost::asio::ip::tcp;
    boost::asio::io_context sensor_io;
    tcp::acceptor acceptor(sensor_io, {tcp::v4(), 0});
    std::vector<uint8_t> packet(16 + 3 * 144, 0);
    packet[0] = 'e'; packet[1] = 'S'; packet[2] = 1;
    const uint32_t sequence = 123;
    const uint32_t length = 4 + 3 * 144;
    const uint16_t frame_size = 144;
    std::memcpy(packet.data() + 4, &sequence, sizeof(sequence));
    std::memcpy(packet.data() + 8, &length, sizeof(length));
    std::memcpy(packet.data() + 14, &frame_size, sizeof(frame_size));
    for (int frame = 0; frame < 3; ++frame) {
        auto set_float = [&](int word, float value) {
            std::memcpy(packet.data() + 16 + frame * 144 + word * 4, &value, sizeof(value));
        };
        set_float(0, 1.0f + frame * 0.001f);
        set_float(15, 20);  // horizontal yaw rate
        set_float(16, 10.8f); // forward km/h
        set_float(17, 0);
    }
    std::thread sensor([&] {
        tcp::socket socket(sensor_io);
        acceptor.accept(socket);
        boost::asio::write(socket, boost::asio::buffer(packet.data(), 5));
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        boost::asio::write(socket, boost::asio::buffer(packet.data() + 5, packet.size() - 5));
    });
    boost::asio::io_context oms_io;
    comms::OMSComms oms(oms_io, "127.0.0.1", acceptor.local_endpoint().port());
    std::thread network([&] { oms_io.run(); });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    estimation::OMSGroundEstimator::State state;
    do {
        state = core::StateTracker::instance().get_latest_state_and_validity().first.oms;
        if (state.sample_time_s >= 1.002f) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    oms_io.stop();
    network.join();
    sensor.join();
    check(state.sample_time_s >= 1.002f && std::abs(state.speed_mps - 3.0f) < 0.001f &&
          std::abs(state.heading_deg - 1.04) < 0.001,
          "Fragmented OMS TCP packets decode all frames and integrate sensor-time heading");
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
        receive_oms(0, 3.6f, 0, 10);
        receive_oms(0.1f, 3.6f, 0, 10);
        const auto optical = tracker.get_latest_state_and_validity().first.oms;
        check(optical.received && std::abs(optical.speed_mps - 1) < 0.001f &&
              std::abs(optical.heading_deg - 1) < 0.001f,
              "OMS protobuf provides ground speed and sample-clock heading to vehicle state");
        check_oms_tcp_stream();

        // Synthetic healthy state isolates controller logic from physical hardware.
        state = {};
        state.state_is_valid = state.is_ready_to_drive = state.wheel_speeds_valid = true;
        core::VehicleControllerManager::create({std::make_shared<IdleController>(), controller, std::make_shared<IdleController>()});
        auto& manager = core::VehicleControllerManager::instance();
        check(manager.init(), "Controller manager initializes");
        ParameterClient client;
        client.set("use_ground_speed", false);
        client.set("heading_control_enabled", false);
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

        client.set("testing_enabled", false);
        client.set("braking_distance_m", 5.0);
        client.set("use_ground_speed", true);
        client.set("heading_control_enabled", true);
        state.oms.received = true;
        state.oms.heading_deg = 30;
        state.oms.distance_m = 100;
        client.set("testing_enabled", true);
        auto ground_step = [&] { return manager.step_active_controller(state); };
        check(ground_step().steering_column_angle_deg == 0.0f, "Ground run captures initial heading");
        state.oms.heading_deg = 32;
        state.oms.yaw_rate_deg_s = 2;
        auto ground_output = ground_step();
        check(ground_output.steering_column_angle_deg == -5.0f, "Heading error commands Kraken column correction");
        const auto equal = std::get<core::SpeedControlOut>(ground_output.out);
        check(equal.desired_rpms.FL == equal.desired_rpms.FR && equal.desired_rpms.FL == equal.desired_rpms.RL &&
              equal.desired_rpms.FL == equal.desired_rpms.RR && equal.torque_lim_nm.FL == equal.torque_lim_nm.FR &&
              equal.torque_lim_nm.FL == equal.torque_lim_nm.RL && equal.torque_lim_nm.FL == equal.torque_lim_nm.RR,
              "Heading correction never uses torque vectoring");
        state.oms.speed_mps = 1;
        state.oms.distance_m = 105;
        state.current_rpms = {2000, 2000, 2000, 2000}; // spinning faster than the ground
        ground_output = ground_step();
        const auto ground_brake = std::get<core::SpeedControlOut>(ground_output.out);
        check(std::abs(ground_brake.desired_rpms.FL - constants::METERS_PER_SECOND_TO_RPM) < 0.01f &&
              ground_brake.torque_lim_nm.FL == 1,
              "Optical distance triggers braking using ground entry speed despite wheel slip");
        check(ground_output.steering_column_angle_deg == -5.0f, "Heading hold continues while braking");
        state.oms.speed_mps = 0;
        state.oms.distance_m = 110;
        state.current_rpms = {};
        ground_output = ground_step();
        check(ground_output.steering_column_angle_deg == 0.0f, "Steering centers when run completes");
        core::VehicleControllerManager::destroy();
    }
    core::StateTracker::destroy();
    core::FoxgloveServer::destroy();
    core::MCAPLogger::destroy();
    std::cout << "Wheel feedback and controller integration checks passed\n";
}
