#include <VNComms.hpp>

// standard includes
#include <iostream>
#include <cstring>
#include <cerrno>
#include <cctype>

#include "hytech_msgs.pb.h"
#include "base_msgs.pb.h"

#include "libvncxx/types.h"
#include "libvncxx/vntime.h"
#include "libvncxx/packetfinder.h"
#include "libvncxx/packet.h"
#include <spdlog/spdlog.h>

#include "FoxgloveServer.hpp"
#include "Telemetry.hpp"

namespace comms
{

    bool VNDriver::init() {
        // Try to establish a connection to the driver
        spdlog::info("Opening vn driver.");
        auto& foxglove = core::FoxgloveServer::instance();

        auto device_name = foxglove.get_param<std::string>("vn_driver/device_name");
        auto baud_rate = foxglove.get_param<int>("vn_driver/baud_rate");
        auto freq_divisor = foxglove.get_param<int>("vn_driver/freq_divisor");
        auto port = foxglove.get_param<int>("vn_driver/port");

        if(!(device_name && baud_rate && freq_divisor && port)) {
            spdlog::error("Couldn't load all params for VN Driver");
            return false;
        }

        _config.baud_rate = baud_rate.value();
        _config.freq_divisor = freq_divisor.value();

        _processor.registerPossiblePacketFoundHandler(this, &VNDriver::_handle_recieve);
        
        boost::system::error_code ec;
        auto ec_ret = _serial.open(device_name.value(), ec);

        if (ec) {
            spdlog::warn("Error: {}", ec.message());
            spdlog::info("failed to open vectornav serial port");
            return false;
        }

        // Set the baud rate of the device along with other configs
        _serial.set_option(SerialPort::baud_rate(_config.baud_rate));
        _serial.set_option(SerialPort::character_size(8));
        _serial.set_option(SerialPort::parity(SerialPort::parity::none));
        _serial.set_option(SerialPort::stop_bits(SerialPort::stop_bits::one));
        _serial.set_option(SerialPort::flow_control(SerialPort::flow_control::none));
        
        // Configures the binary outputs for the device
        spdlog::info("Configuring binary outputs.");
        _configure_binary_outputs();

        // spdlog::info("INS config");
        // TODO: uncomment
        // _configure_INS();
        
        
        return true;
    }

    VNDriver::VNDriver(boost::asio::io_context& io, bool &init_not_successful) : _serial(io) {

        init_not_successful = !init();
        // Starts read
        if (!init_not_successful) {
            spdlog::info("Starting vn driver recieve.");
            _start_recieve();
        } 
        
    }

    void VNDriver::log_proto_message(std::shared_ptr<google::protobuf::Message> msg) {
        core::log(static_cast<std::shared_ptr<google::protobuf::Message>>(msg));
        core::StateTracker::instance().handle_receive_protobuf_message(msg);
    }


    // TODO: FINISH IMPLEMENTATION
    // Sets the heading estimate to the angle provided by the user, immediately initializing the INS filter and expediting the startup process
    //      Must be before INS mode 3
    //      Have to feed initial heading within 5 degrees of True North
    //      Calculate declination (w/Reg21 & 83) and offset Magnetic North to get True North
    //      Write result to Reg 161 while VN-300 in Mode 0 to set initial heading and expediate startup to Mode 3
    // New method to try and get initial heading from the VN-300
    void VNDriver::_try_initialize_heading(float mag_heading, uint8_t ins_mode) {
        if (_initial_heading_set) return; 
    
        if (ins_mode == 0) {
            float true_heading = mag_heading + _local_declination;

            _set_initial_heading(true_heading);

            _initial_heading_set = true;
            spdlog::info("Applied declination of {} deg for True Heading of {}", _local_declination, true_heading);
        }
        return;
    }

    // Sets the heading estimate to the angle provided by the user, immediately initializing the INS filter and expediting the startup process
    void VNDriver::_set_initial_heading(float initial_heading) {
        // Packet namespace does not have set intial heading command. Generate the $VNSIH command.
        // The manual shows the format: $VNSIH,heading
        #if VN_HAVE_SECURE_CRT
        size_t length = sprintf_s((char*)_output_buff.data(), _output_buff.size(), "$VNSIH,%.3f", initial_heading);
        #else
        size_t length = sprintf((char*)_output_buff.data(), "$VNSIH,%.3f", initial_heading);
        #endif

        // Create packet Checksum
        auto num_of_bytes = Packet::finalizeCommand(ErrorDetectionMode::ERRORDETECTIONMODE_NONE, (char*)_output_buff.data(), length);

        // Send command to VN-300
        boost::asio::async_write(_serial,
            boost::asio::buffer(_output_buff.data(), num_of_bytes),
            [](const boost::system::error_code &ec, std::size_t bytes_transferred) {
            if (!ec) {
                spdlog::info("Successfully sent {} bytes.", bytes_transferred);
            } else {
                spdlog::error("Error sending data: {}", ec.message());
            }
        });
    }


    void VNDriver::_configure_INS() {
        
        // TODO: Separate function for this?? Consider if VN-300 is ARHS-enabled by default??
        // Creating buffer for writing to INS Basic Configuration (Register 67)
        auto num_of_bytes = Packet::genWriteInsBasicConfiguration(
            ErrorDetectionMode::ERRORDETECTIONMODE_NONE,
            (char *)_output_buff.data(),
            _output_buff.size(),
            0, // Scenario: AHRS (Attitude Only - no GNSS/INS)
            1, // Set AhrsAiding: provides the ability to switch to using the magnetometer to stabilize heading during times when GNSS-based heading is unavailable.
            0  // Estimate Baseline: if enabled, the sensor will auto-populate the GNSS Compass Estimated Baseline register (Register 97) with the INS-estimated baseline
        );

        // Write the buffer to the VN-300
        boost::asio::async_write(_serial,
            boost::asio::buffer(_output_buff.data(), num_of_bytes),
            [](const boost::system::error_code &ec, std::size_t bytes_transferred) {
            if (!ec) {
                spdlog::info("Successfully sent {} bytes.", bytes_transferred);
            } else {
                spdlog::error("Error sending data: {}", ec.message());
            }
        });

        // TODO: SET WNN HERE
    }

    void VNDriver::_configure_binary_outputs() {
        
        auto num_of_bytes = Packet::genWriteBinaryOutput1(
            ErrorDetectionMode::ERRORDETECTIONMODE_NONE,
            (char *)_output_buff.data(),
            _output_buff.size(),
            AsyncMode::ASYNCMODE_PORT1,
            _config.freq_divisor,
            (CommonGroup::COMMONGROUP_YAWPITCHROLL | CommonGroup::COMMONGROUP_ANGULARRATE), // Note use of binary OR to configure flags.
            TimeGroup::TIMEGROUP_NONE,
            ImuGroup::IMUGROUP_UNCOMPACCEL,
            GpsGroup::GPSGROUP_NONE,
            AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY,
            (InsGroup::INSGROUP_INSSTATUS | InsGroup::INSGROUP_POSLLA | InsGroup::INSGROUP_VELBODY | InsGroup::INSGROUP_VELU),
            GpsGroup::GPSGROUP_NONE);
            
            boost::asio::async_write(_serial,
                boost::asio::buffer(_output_buff.data(), num_of_bytes),
                [](const boost::system::error_code &ec, std::size_t bytes_transferred) {
                if (!ec) {
                    spdlog::info("Successfully sent {} bytes.", bytes_transferred);
                } else {
                    spdlog::error("Error sending data: {}", ec.message());
                }
            });
    }

    void VNDriver::_handle_recieve(void *userData, vn::protocol::uart::Packet &packet, size_t runningIndexOfPacketStart, TimeStamp ts) {
        auto this_instance = (VNDriver *)userData;
        if (packet.type() == vn::protocol::uart::Packet::TYPE_BINARY) {
            vn::math::vec3f vel;
            // See if this is a binary packet type we are expecting.
            if (!packet.isCompatible((CommonGroup::COMMONGROUP_YAWPITCHROLL | CommonGroup::COMMONGROUP_ANGULARRATE), // Note use of binary OR to configure flags.
            TimeGroup::TIMEGROUP_NONE,
            ImuGroup::IMUGROUP_UNCOMPACCEL,
            GpsGroup::GPSGROUP_NONE,
            AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY,
            (InsGroup::INSGROUP_INSSTATUS | InsGroup::INSGROUP_POSLLA | InsGroup::INSGROUP_VELBODY | InsGroup::INSGROUP_VELU),
            GpsGroup::GPSGROUP_NONE)) {
                spdlog::warn("ERROR: packet format invalid");
                return;
            }

            // Extract data in correct order
            auto ypr_data = packet.extractVec3f();
            auto angular_rate_data = packet.extractVec3f();
            auto uncomp_accel = packet.extractVec3f();
            auto linear_accel_body = packet.extractVec3f();
            uint16_t ins_status = packet.extractUint16();
            auto pos_lla = packet.extractVec3d();
            auto vel_body = packet.extractVec3f();
            auto vel_uncertainty = packet.extractFloat();
            
            auto ins_mode = static_cast<hytech_msgs::INSMode>(ins_status & 0b11); // Extract INS mode from status

            // TODO: Attempt to set initial heading
            // _try_initialize_heading(ypr_data.x, ins_mode);

            // Create the protobuf message to send
            std::shared_ptr<hytech_msgs::VNData> msg_out = std::make_shared<hytech_msgs::VNData>();
            
            hytech_msgs::xyz_vector *linear_vel_msg = msg_out->mutable_vn_vel_m_s();
            linear_vel_msg->set_x(vel_body.x);
            linear_vel_msg->set_y(vel_body.y);
            linear_vel_msg->set_z(vel_body.z);
            
            hytech_msgs::xyz_vector *linear_accel_msg = msg_out->mutable_vn_linear_accel_m_ss();
            linear_accel_msg->set_x(linear_accel_body.x);
            linear_accel_msg->set_y(linear_accel_body.y);
            linear_accel_msg->set_z(linear_accel_body.z);

            hytech_msgs::xyz_vector *linear_accel_uncomp_msg = msg_out->mutable_vn_linear_accel_uncomp_m_ss();
            linear_accel_uncomp_msg->set_x(uncomp_accel.x);
            linear_accel_uncomp_msg->set_y(uncomp_accel.y);
            linear_accel_uncomp_msg->set_z(uncomp_accel.z);

            hytech_msgs::xyz_vector *angular_rate_data_msg = msg_out->mutable_vn_angular_rate_rad_s();
            angular_rate_data_msg->set_x(angular_rate_data.x);
            angular_rate_data_msg->set_y(angular_rate_data.y);
            angular_rate_data_msg->set_z(angular_rate_data.z);

            hytech_msgs::ypr_vector *ypr_data_msg = msg_out->mutable_vn_ypr_rad();
            ypr_data_msg->set_yaw(ypr_data.x);
            ypr_data_msg->set_pitch(ypr_data.y);
            ypr_data_msg->set_roll(ypr_data.z);

            hytech_msgs::GPS_data *vn_gps_msg = msg_out->mutable_vn_gps();
            vn_gps_msg->set_lat(pos_lla.x);
            vn_gps_msg->set_lon(pos_lla.y);

            hytech_msgs::vn_status *vn_ins_msg = msg_out->mutable_status();
            vn_ins_msg->set_ins_mode(ins_mode); 
            vn_ins_msg->set_gnss_fix((ins_status >> 2) & 0b1); 
            vn_ins_msg->set_error_imu((ins_status >> 4) & 1);
            vn_ins_msg->set_error_mag_pres((ins_status >> 5) & 0b1);
            vn_ins_msg->set_error_gnss((ins_status >> 6) & 0b1);
            vn_ins_msg->set_gnss_heading_ins((ins_status >> 8) & 0b1); 
            vn_ins_msg->set_gnss_compass((ins_status >> 9) & 0b1); 
            vn_ins_msg->set_ins_mode_int((ins_status & 0b11));
            vn_ins_msg->set_ins_vel_u(vel_uncertainty);

            this_instance->log_proto_message(static_cast<std::shared_ptr<google::protobuf::Message>>(msg_out));
        } else {
            spdlog::warn("Packet not correct. Are you sure the vectornav is configured to only send binary outputs?");
        }
    }

    void VNDriver::_start_recieve() {
        _serial.async_read_some(
            boost::asio::buffer(_input_buff),
            [&](const boost::system::error_code &ec, std::size_t bytesCount) -> void
            {
                if (ec) {
                    if (ec != boost::asio::error::operation_aborted) {
                        spdlog::error("ERROR: {}", ec.message());
                    }
                    return;
                }
                // _logger.log_string("logging", core::LogLevel::INFO);
                _processor.processReceivedData((char *)(_input_buff.data()), bytesCount);
                // Initiate another asynchronous read
                _start_recieve();
            });
    }

}
