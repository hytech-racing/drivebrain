#include <StateTracker.hpp>
#include <LaunchController.hpp>
#include <TeleopController.hpp>
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
#include <fstream>
#include <limits>
#include <linux/joystick.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

void check(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
void check_zero_drive(const core::ControllerOutput& output, const char* message) {
    const auto& drive = std::get<core::SpeedControlOut>(output.out);
    check(drive.desired_rpms.FL == 0 && drive.desired_rpms.FR == 0 &&
          drive.desired_rpms.RL == 0 && drive.desired_rpms.RR == 0 &&
          drive.torque_lim_nm.FL == 0 && drive.torque_lim_nm.FR == 0 &&
          drive.torque_lim_nm.RL == 0 && drive.torque_lim_nm.RR == 0, message);
}

// Feed real Linux joydev events through the existing Xbox reader, without hardware.
class BenchJoystick {
public:
    explicit BenchJoystick(const char* original_config) {
        char pattern[] = "/tmp/drivebrain-xbox-XXXXXX";
        const char* directory = mkdtemp(pattern);
        check(directory, "Create temporary joystick directory");
        _directory = directory;
        _device = _directory + "/js0";
        config_path = _directory + "/config.json";
        check(mkfifo(_device.c_str(), 0600) == 0, "Create joystick event pipe");
        _writer = open(_device.c_str(), O_RDWR | O_NONBLOCK);
        check(_writer >= 0, "Open joystick event writer");
        nlohmann::json config;
        std::ifstream(original_config) >> config;
        config["TeleopController"]["joystick_device_path"] = _device;
        std::ofstream(config_path) << config;
    }
    ~BenchJoystick() {
        disconnect();
        unlink(_device.c_str());
        unlink(config_path.c_str());
        rmdir(_directory.c_str());
    }
    void axis(uint8_t number, int16_t value) {
        const js_event event{0, value, JS_EVENT_AXIS, number};
        check(write(_writer, &event, sizeof(event)) == sizeof(event), "Write joystick axis event");
    }
    void wait_axis(const control::TeleopController& controller, float expected) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline) {
            const auto input = controller.get_steering_input();
            if (input && std::abs(*input - expected) < 0.001f) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        check(false, "Xbox axis event reaches shared reader");
    }
    void disconnect() {
        // A disconnected joydev device also disappears until it is plugged in again.
        unlink(_device.c_str());
        if (_writer >= 0) { close(_writer); _writer = -1; }
    }
    std::string config_path;
private:
    std::string _directory, _device;
    int _writer = -1;
};
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
    BenchJoystick joystick(argv[1]);
    core::MCAPLogger::create("/tmp/", mcap::McapWriterOptions(""), joystick.config_path);
    core::FoxgloveServer::create(joystick.config_path);
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
        auto teleop = std::make_shared<control::TeleopController>();
        check(teleop->init(), "Xbox reader initializes");
        joystick.wait_axis(*teleop, 0);
        auto controller = std::make_shared<control::LaunchController>(
            [teleop] { return teleop->get_steering_input(); },
            [teleop] { return teleop->get_steering_column_target(); });
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
        core::VehicleControllerManager::create({std::make_shared<IdleController>(), teleop, controller});
        auto& manager = core::VehicleControllerManager::instance();
        check(manager.init(), "Controller manager initializes");
        check(manager.get_active_controller_index() == 0, "Original default controller zero is preserved");
        check(core::TELEOP_CONTROLLER_INDEX == 1 && core::LAUNCH_CONTROLLER_INDEX == 2,
              "Preserve old teleop index and append launch at a higher index");
        check(manager.swap_active_controller(core::LAUNCH_CONTROLLER_INDEX, state), "Select new launch controller");
        ParameterClient client;
        auto step = [&] { return std::get<core::SpeedControlOut>(manager.step_active_controller(state).out); };
        client.set("testing_enabled", true);
        check(step().torque_lim_nm.FL == 0 && !manager.step_active_controller(state).steering_enabled,
              "Default stage zero disables drivetrain and steering even when testing enabled");
        client.set("testing_enabled", false);
        client.set("test_stage", 1);
        state = {}; // Direct steering needs neither RTD/wheel feedback nor OMS.
        joystick.axis(0, -32767);
        joystick.wait_axis(*teleop, -1);
        client.set("testing_enabled", true);
        auto steering_only = manager.step_active_controller(state);
        check(steering_only.steering_enabled && steering_only.steering_column_angle_deg == 70,
              "Stage one restores original Xbox left steering and its 70 degree limit");
        check_zero_drive(steering_only, "Steering-only stage never commands any drive motor");
        joystick.axis(5, -32767); // arm then squeeze accelerator
        joystick.axis(5, 32767);
        joystick.axis(2, -32767); // arm then squeeze brake
        joystick.axis(2, 32767);
        joystick.axis(0, 32767); // also serves as a barrier after trigger events
        joystick.wait_axis(*teleop, 1);
        steering_only = manager.step_active_controller(state);
        check(steering_only.steering_column_angle_deg == -70, "Direct Xbox right steering remains rightward");
        check_zero_drive(steering_only, "Xbox triggers cannot spin drive motors in steering-only stage");
        joystick.axis(0, 6553);
        joystick.wait_axis(*teleop, 0.2f);
        steering_only = manager.step_active_controller(state);
        check(std::abs(*steering_only.steering_column_angle_deg + 0.56f) < 0.001f &&
              steering_only.steering_column_angle_deg == teleop->step_controller(state).steering_column_angle_deg,
              "Stage one shares the exact original cubic teleop steering response");
        client.set("testing_enabled", false);
        check(!manager.step_active_controller(state).steering_enabled, "Testing false disables direct Xbox steering");
        state.state_is_valid = state.is_ready_to_drive = state.wheel_speeds_valid = true;
        check(manager.swap_active_controller(core::TELEOP_CONTROLLER_INDEX, state), "Original teleop remains at index one");
        joystick.axis(2, -32767); // release brake, keep accel fully squeezed
        joystick.axis(0, -32767);
        joystick.wait_axis(*teleop, -1);
        auto manual = manager.step_active_controller(state);
        check(std::get<core::SpeedControlOut>(manual.out).desired_rpms.FL == 500 &&
              std::get<core::SpeedControlOut>(manual.out).torque_lim_nm.FL == 4 &&
              manual.steering_column_angle_deg == 70,
              "Original index-one teleop retains both drive and steering behavior");
        joystick.axis(5, -32767);
        joystick.axis(0, 0);
        joystick.wait_axis(*teleop, 0);
        client.set("test_stage", 2);
        check(manager.swap_active_controller(0, state), "Select idle controller to test pending start");
        client.set("testing_enabled", true);
        check(manager.swap_active_controller(core::LAUNCH_CONTROLLER_INDEX, state), "Select launch controller");
        check(step().torque_lim_nm.FL == 0, "Selecting controller discards pending launch");
        client.set("testing_enabled", false);
        client.set("testing_enabled", true);
        check(step().torque_lim_nm.FL == 2, "False-to-true edge starts launch");
        check(manager.step_active_controller(state).steering_column_angle_deg == 0.0f,
              "Stage two holds straight with no OMS data");
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
        state.current_rpms = {1000, 1000, 1000, 1000};
        client.set("testing_enabled", false);
        auto disabled = manager.step_active_controller(state);
        check(std::get<core::SpeedControlOut>(disabled.out).desired_rpms.FL == 0 &&
              std::get<core::SpeedControlOut>(disabled.out).torque_lim_nm.FL == 0 && !disabled.steering_enabled,
              "Disabling immediately clears speed, torque and Kraken enable while wheels are moving");
        state.current_rpms = {};
        client.set("braking_distance_m", 0.0);
        client.set("testing_enabled", true);
        check(step().torque_lim_nm.FL == 0, "Invalid configuration cannot start a run");

        client.set("testing_enabled", false);
        client.set("braking_distance_m", 5.0);
        client.set("test_stage", 3);
        state.oms = {};
        state.oms.heading_deg = std::numeric_limits<double>::quiet_NaN();
        state.oms.speed_mps = std::numeric_limits<float>::quiet_NaN();
        state.oms.distance_m = std::numeric_limits<double>::quiet_NaN();
        client.set("testing_enabled", true);
        check(step().torque_lim_nm.FL == 2, "Xbox stage starts without any OMS data");
        joystick.axis(0, -6553);
        joystick.wait_axis(*teleop, -0.2f);
        auto xbox_output = manager.step_active_controller(state);
        check(xbox_output.steering_enabled && std::abs(*xbox_output.steering_column_angle_deg + 4) < 0.01f,
              "Moving Xbox left stick slightly left commands right correction");
        state.oms.received = true;
        state.oms.heading_deg = 180;
        state.oms.distance_m = 1000000;
        state.oms.speed_mps = 50;
        joystick.axis(0, 6553);
        joystick.wait_axis(*teleop, 0.2f);
        xbox_output = manager.step_active_controller(state);
        check(std::abs(*xbox_output.steering_column_angle_deg - 4) < 0.01f &&
              std::get<core::SpeedControlOut>(xbox_output.out).torque_lim_nm.FL == 2,
              "Right stick deflection corrects left and stage three ignores live OMS heading/travel");
        const auto xbox_drive = std::get<core::SpeedControlOut>(xbox_output.out);
        check(xbox_drive.desired_rpms.FL == xbox_drive.desired_rpms.FR &&
              xbox_drive.desired_rpms.FL == xbox_drive.desired_rpms.RL &&
              xbox_drive.desired_rpms.FL == xbox_drive.desired_rpms.RR &&
              xbox_drive.torque_lim_nm.FL == xbox_drive.torque_lim_nm.FR &&
              xbox_drive.torque_lim_nm.FL == xbox_drive.torque_lim_nm.RL &&
              xbox_drive.torque_lim_nm.FL == xbox_drive.torque_lim_nm.RR,
              "Fake heading correction never changes individual wheel drive commands");
        joystick.axis(0, -32767);
        joystick.wait_axis(*teleop, -1);
        check(manager.step_active_controller(state).steering_column_angle_deg == -15,
              "Simulated heading respects steering correction limit");
        joystick.axis(0, 1000);
        joystick.wait_axis(*teleop, 0);
        check(manager.step_active_controller(state).steering_column_angle_deg == 0,
              "Xbox deadzone and centered stick return correction to straight");
        joystick.axis(0, -6553);
        joystick.wait_axis(*teleop, -0.2f);
        client.set("testing_enabled", false);
        disabled = manager.step_active_controller(state);
        check(!disabled.steering_enabled && std::get<core::SpeedControlOut>(disabled.out).torque_lim_nm.FL == 0 &&
              !controller->preview_controller(state).steering_enabled,
              "Testing false clears fake-heading steering and drivetrain, including previews");
        client.set("testing_enabled", true);
        check(step().torque_lim_nm.FL == 2, "Xbox stage restarts on a fresh enable edge");
        state.current_rpms = {1000, 1000, 1000, 1000};
        client.set("brake_requested", true);
        xbox_output = manager.step_active_controller(state);
        check(std::get<core::SpeedControlOut>(xbox_output.out).torque_lim_nm.FL == 1 &&
              std::abs(std::get<core::SpeedControlOut>(xbox_output.out).desired_rpms.FL - 1000) < 0.01f &&
              *xbox_output.steering_column_angle_deg < 0,
              "Xbox stage brakes from measured wheel speed and continues heading correction");
        client.set("test_stage", 0);
        disabled = manager.step_active_controller(state);
        check(std::get<core::SpeedControlOut>(disabled.out).torque_lim_nm.FL == 0 && !disabled.steering_enabled,
              "Stage zero immediately cancels an active run");
        client.set("test_stage", 3);
        state.current_rpms = {};
        client.set("brake_requested", false);
        check(step().torque_lim_nm.FL == 0, "Changing stage while enabled never starts a run");
        client.set("testing_enabled", false);
        client.set("test_stage", 99);
        client.set("testing_enabled", true);
        check(step().torque_lim_nm.FL == 0, "Invalid stage cannot start motors");
        client.set("testing_enabled", false);
        client.set("test_stage", 4);
        state.oms = {};
        client.set("testing_enabled", true);
        check(step().torque_lim_nm.FL == 0, "Ground stage still requires OMS data");
        client.set("testing_enabled", false);
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
        check(ground_output.steering_column_angle_deg == 0.0f && !ground_output.steering_enabled,
              "Kraken is neutral when run completes");
        client.set("testing_enabled", false);
        client.set("test_stage", 3);
        joystick.disconnect();
        const auto disconnect_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (teleop->get_steering_input() && std::chrono::steady_clock::now() < disconnect_deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        check(!teleop->get_steering_input(), "Xbox disconnect is visible to launch controller");
        client.set("testing_enabled", true);
        check(step().torque_lim_nm.FL == 0 && !manager.step_active_controller(state).steering_enabled,
              "Xbox stage cannot run without a connected controller");
        core::VehicleControllerManager::destroy();
    }
    core::StateTracker::destroy();
    core::FoxgloveServer::destroy();
    core::MCAPLogger::destroy();
    std::cout << "Wheel feedback and controller integration checks passed\n";
}
