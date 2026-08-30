#pragma once

#include <cstdint>
#include <optional>
#include <foxglove/PointCloud.pb.h>
#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>
#include <vector> 
#include <string> 
#include <fstream> 
#include <thread>
#include <spdlog/spdlog.h>

#include "hytech.pb.h"
#include "hytech_msgs.pb.h"
#include "dv_msgs.pb.h"

/**
 * The state tracker acts as a thread-safe translation unit
 * between our comms interfaces and the rest of the program. It keeps track 
 * of current internal vehicle state, and observed external state.
 */
namespace core {

    /**
     * @struct Represents a vehicle vector (4 elements of any type)
     */
    template <typename T>
    struct veh_vec {
        T FL;
        T FR;
        T RL;
        T RR;

        template<size_t ind>
        void set_from_index(T val) {
            static_assert(ind <= 3, "ERROR: index cannot be greater than 3");
            if constexpr (ind == 0) {
                FL = val;
            } else if(ind == 1) {
                FR = val;
            } else if(ind == 2) {
                RL = val;
            } else {
                RR = val;
            }
        }
        
        template <size_t ind>
        T get_from_index() {
            static_assert(ind <= 3, "ERROR: index cannot be greater than 3");
            if constexpr (ind == 0) {
                return FL;
            } else if(ind == 1) {
                return FR;
            } else if(ind == 2) {
                return RL;
            } else {
                return RR;
            }
        }

    };

    /**
     * @struct Represents an xyz vector (3 elements of any type)
     */
    template <typename T>
    struct xyz_vec {
        T x;
        T y;
        T z;
    };

    /**
     * @struct Represents an xy vector (2 elements of any type)
     */
    template <typename T>
    struct xy_vec {
        T x;
        T y;
    };

    /**
     * @struct Represents a ypr vector (3 elements of any type that represent angle)
     */
    template <typename T>
    struct ypr_vec  {   
        T yaw;
        T pitch;
        T roll;
    };

    /**
     * @struct Represents raw sensor data
     */
    struct RawInputData  {
        veh_vec<float> raw_load_cell_values;
        veh_vec<float> raw_shock_pot_values;
        float raw_steering_analog;
        float raw_steering_digital;
        veh_vec<float> raw_inverter_torques;
        veh_vec<float> raw_inverter_power;
    };

    /**
     * @struct position
     */
    struct Position {
        double lat;
        double lon;
        bool valid;
    };

    /**
     * @struct A teleop command from the Foxglove teleop panel, as a Twist.
     * Forward speed and yaw rate get mapped to torque and steering by the app.
     */
    struct TeleopCommand {
        float linear_x;
        float angular_z;
        std::chrono::steady_clock::time_point recv_time;
    };

    /**
     * @struct Contains requested pedals information
     */
    struct DriverInput {
        float requested_accel; // float from 0 to 1 representing percent of accel pedal travel
        float requested_brake; // float from 0 to 1 representing pedal travel of brake
    };

    /**
     * @struct Contains the torque outputs of the controller
     */
    struct ControllerTorqueOut {
        veh_vec<float> res_torque_lim_nm;
    };

    /**
     * @struct Represents the outputs of the tire model 
     */
    struct TireDynamics {
        veh_vec<xyz_vec<float>> tire_forces_n;
        veh_vec<xyz_vec<float>> tire_moments_nm;
        veh_vec<float> accel_saturation_nm;
        veh_vec<float> brake_saturation_nm;
        float v_y_lm;
        float psi_dot_lm_rad_s;
        TireDynamics() {
            tire_forces_n = {};
            tire_moments_nm = {};
            accel_saturation_nm = {};
            brake_saturation_nm = {};
            v_y_lm = 0;
            psi_dot_lm_rad_s = 0;
        }
    };

    /**
     * @struct Represents more specific outputs of the controller 
     * re. torque vectoring statuses
     */
    struct TorqueVectoringStatus {
        veh_vec<float> torque_additional_nm;
        float additional_mz_moment_nm;
        float des_psi_dot;
        float psi_dot_err;
        float perceived_vx;
        float integral_yaw_rate_err;
        float perceived_psi_dot;
    };

    /**
     * @struct Collection of different outputs
     */
    struct MatlabMathResult {
        TireDynamics tire_dynamics;
        TorqueVectoringStatus torque_vectoring_status;
    };

    /**
     * @struct Collection of desired speed and torques
     */
    struct SpeedControlOut {
        int64_t mcu_recv_millis;
        veh_vec<float> desired_rpms;
        veh_vec<float> torque_lim_nm;
    };

    /**
     * @struct A specific torque control out
     */
    struct TorqueControlOut {
        veh_vec<float> desired_torques_nm;
    };


    /**
     * @struct A controller output
     */
    struct ControllerOutput {
        std::variant<SpeedControlOut, TorqueControlOut, std::monostate> out;
        // To only be set in a driverless context
        std::optional<float> desired_steering_deg;
    };

    /**
     * @struct Contains the health of our ins 
     */
    struct INSStatus {
        float vel_uncertainty;
        int status_mode;
    };

    /**
     * @struct Contains drivetrain information
     */
    struct DrivetrainData {
        veh_vec<float> inverter_igbt_temps_c;
        veh_vec<float> inverter_temps_c;
        veh_vec<float> inverter_motor_temps_c;
    };

    /**
     * @struct Contains accumulator information
     */
    struct AccumulatorData {
        float min_cell_voltage;
        float pack_voltage;
    };

    /**
     * @struct Contains a wealth of data
     * representing the entire internal 
     * state of the vehicle. 
     */
    struct VehicleState {
        bool is_ready_to_drive; 
        DriverInput input;
        xyz_vec<float> current_body_vel_ms;
        xyz_vec<float> current_body_accel_mss;
        xyz_vec<float> current_angular_rate_rads;
        ypr_vec<float> current_ypr_rad;
        veh_vec<float> current_rpms;
        veh_vec<float> motor_overload_percentages;
        bool state_is_valid;
        int prev_MCU_recv_millis;
        float steering_angle_deg;
        ControllerOutput prev_controller_output;
        TireDynamics tire_dynamics;
        veh_vec<float> driver_torque;
        ControllerTorqueOut matlab_math_temp_out;
        veh_vec<float> suspension_potentiometers_mm;
        Position vehicle_position;
        veh_vec<float> loadcells;
        veh_vec<float> current_torques_nm;
        INSStatus ins_status;
        float old_energy_meter_kw;
        DrivetrainData dt_data;
        AccumulatorData acc_data;
    };

    enum class ControllerManagerStatus
    {
        NO_ERROR = 0,
        ERROR_CONTROLLER_INDEX_OUT_OF_RANGE = 1,
        ERROR_SPEED_TOO_HIGH = 2,
        ERROR_TORQUE_TOO_HIGH = 3,
        ERROR_DRIVER_ON_PEDAL = 4,
        ERROR_CONTROLLER_NO_TORQUE_OR_SPEED_OUTPUT = 5,
        ERROR_OUTPUT_EXCEEDS_PHYS_LIMITS = 6,
        ERROR_REQUESTING_SAME_CTR_TYPE = 7,
        NUM_CONTROLLER_MANAGER_STATUSES = 8,
        ERROR_NULLPTR_CONTROLLER = 9
    };

    struct ControllerManagerState
    {
        ControllerManagerStatus current_status;
        ControllerOutput current_controller_output;
    };

    struct LidarPoint { float x, y, z; uint8_t r, g, b, a; };
    static_assert(sizeof(LidarPoint) == 16);
    inline const LidarPoint* points(const foxglove::PointCloud& pc) { return reinterpret_cast<const LidarPoint*>(pc.data().data()); }
    inline uint64_t num_points(const foxglove::PointCloud& pc) { return pc.data().size() / sizeof(LidarPoint); }


    /**
     * @struct Contains driverless data representing the entire internal
     * and external state of the vehicle
     */
    struct DriverlessState {
        std::shared_ptr<const foxglove::PointCloud> lidar_cloud;
        bool lidar_is_valid;
        std::shared_ptr<const dv_msgs::Cones> cone_observations;
        std::shared_ptr<const std::vector<xyz_vec<float>>> path;

        /* Returns the latest LiDAR point cloud in a clean struct */
        const LidarPoint* points() const { return lidar_cloud ? core::points(*lidar_cloud) : nullptr; }
        uint64_t num_points() const { return lidar_cloud ? core::num_points(*lidar_cloud) : 0; }
        std::shared_ptr<const hytech_msgs::pose> vehicle_pose;
    };


    /**
     * Allows different communications interfaces
     * to update the internal state of drivebrain in a thread-safe manner
     */
    class StateTracker {

        public: 

            /**
             * Constructor for initializing a new StateTracker singleton instance
             */
            static void create(); 

            /**
             * Fetches the StateTracker singleton instance
             */
            static StateTracker& instance(); 

            /* Destroys the StateTracker singleton instance */
            static void destroy(); 

            /* Destructs an instnace of the StateTracker */
            ~StateTracker() {
                spdlog::info("Destructing state tracker");

                _s_instance.store(nullptr, std::memory_order_release);
                spdlog::info("State tracker instance released");
            }

            /**
             * Receives a protobuf message and adds any useful information to the internal 
             * vehicle state. 
             * 
             * @param message the message with information to add. 
             */
            void handle_receive_protobuf_message(std::shared_ptr<google::protobuf::Message> message);
            
            /**
             * Sets the previous control output of the controller
             * 
             * @param previous_control_output the previous output of the controller
             */
            void set_previous_control_output(ControllerOutput &previous_control_output);
            
            /**
             * Returns a pair of the current vehicle state and whether or not it's valid given the timestamps
             * 
             * @return the vehicle state and whether or not its valid. 
             */
            std::pair<core::VehicleState, bool> vehicle_state();

            /**
             * Returns the current driverless state. Consumers judge freshness themselves off
             * the recv times in the struct, since each one wants its own staleness threshold.
             *
             * @return the driverless state.
             */
            DriverlessState dv_state();

            /**
             * Sets the latest planned path produced by the autonomy stack.
             *
             * @param path the planned path, map frame
             */
            void set_dv_path(std::shared_ptr<const std::vector<xyz_vec<float>>> path);


            /**
             * Sets the latest vehicle pose produced by the autonomy stack.
             *
             * @param pose the latest vehicle pose, map frame
             */
            void set_dv_pose(std::shared_ptr<const hytech_msgs::pose> pose);

            /**
             * Sets the latest cone observations produced by perception.
             *
             * @param cones the observed cones, vehicle frame
             */
            void set_cone_observations(std::shared_ptr<const dv_msgs::Cones> cones);

            /**
             * Sets the latest teleop command received from the Foxglove teleop panel.
             *
             * @param command the teleop command
             */
            void set_teleop_command(const TeleopCommand& command);

            /**
             * Returns the latest teleop command.
             *
             * @return the teleop command, and whether it is recent enough to act on
             */
            std::pair<TeleopCommand, bool> teleop_command();


        private:

            template <size_t index, typename inverter_dynamics_message>
            void _handle_set_inverter_dynamics(std::shared_ptr<google::protobuf::Message> msg);

            template <size_t index, typename inverter_temps_message>
            void _handle_set_inverter_temps(std::shared_ptr<google::protobuf::Message> msg);

            template<size_t arr_len> 
            bool _validate_timestamps(const std::array<std::chrono::microseconds, arr_len> &timestamp_arr);

            void _receive_low_level_state(std::shared_ptr<google::protobuf::Message> message);

            void _receive_inverter_states(std::shared_ptr<google::protobuf::Message> message);

            VehicleState _vehicle_state = { };
            RawInputData _raw_input_data = { };
            std::mutex _state_mutex;
            std::array<std::chrono::microseconds, 4> _timestamp_array;

            DriverlessState _dv_state = { };
            std::mutex _dv_state_mutex;

            TeleopCommand _teleop_command = { };
            std::mutex _teleop_mutex;
            
            /* Private constructor called by the init method */
            StateTracker() {}; 
            
            /* Singleton move semantics */
            StateTracker(const StateTracker&) = delete; 
            StateTracker& operator=(const StateTracker&) = delete;

            /* Singleton instance */
            inline static std::atomic<StateTracker*> _s_instance; 

    };
}
