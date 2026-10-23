#include "ETHRecvComms.hpp"
#include "OMSComms.hpp"
#include "CANComms.hpp"
#if HOOTL_ENABLED
# include "SimComms.hpp"
#else
# include "KrakenComms.hpp"
#endif
#include <MCAPLogger.hpp>
#include <boost/asio.hpp>
#include <memory>
#include "LoadCellTorqueController.hpp"
#include "TeleopController.hpp"
#include "VNComms.hpp"
#include "OusterComms.hpp"
#include "autonomy.hpp"

#include "hytech_msgs.pb.h"
#include <BlackflyComms.hpp>

enum class DrivingMode { DRIVERED, DRIVERLESS, TELEOP, REMOTE };

class DrivebrainApp {
public:
  /**
   * Creates a new instance of Drivebrain
   *
   * @param json_params_path The path to the JSON parameters file
   * @param dbc_path The path to the DBC file
  */
   DrivebrainApp(const std::string& json_params_path, const std::string& dbc_path);
  ~DrivebrainApp();

  DrivebrainApp& operator=(DrivebrainApp&&) = delete;
  DrivebrainApp(DrivebrainApp&&) = delete;

  /**
   * Starts running the Drivebrain main loop
  */
  void run();

private:
  /**
   * Processes one loop iteration of Drivebrain
  */
  void _loop();

  /**
   * Sends a controller output to the inverters and steering over both CAN buses,
   * and logs what was sent.
   *
   * @param out the controller output to command
  */
  void _actuate(const core::ControllerOutput& out);

  /**
   * Maps a teleop twist onto torque and steering.
   *
   * @param command the latest teleop command
   * @return the controller output to send
  */
  core::ControllerOutput _teleop_command(const core::TeleopCommand& command);

  /**
   * Maps a remote speed command onto a speed controller output. A stale command
   * maps to zero rpm and zero torque limit, and torque limits are capped on the car.
   *
   * @param command the latest remote command
   * @param is_fresh whether the command is recent enough to act on
   * @param vehicle_state the latest vehicle state
   * @return the controller output to send
  */
  core::ControllerOutput _remote_command(const core::RemoteCommand& command, bool is_fresh, const core::VehicleState& vehicle_state);

private:
  boost::asio::io_context _io_context;

  /* Threads */
  std::thread _loop_thread;
  std::thread _io_context_thread;

  /* Config paths */
  const std::string _json_params_path;
  const std::string _dbc_path;

  /* CAN drivers */
  std::unique_ptr<comms::CANComms> _telem_can;
  std::unique_ptr<comms::CANComms> _aux_can;

  /* Ethernet drivers */
  std::unique_ptr<comms::ETHRecvComms<hytech_msgs::ACUAllData>> _acu_eth_driver;
  std::unique_ptr<comms::ETHRecvComms<hytech_msgs::ACUCoreData>> _acu_core_eth_driver;
  std::unique_ptr<comms::ETHRecvComms<hytech_msgs::VCRData_s>> _vcr_eth_driver;
  std::unique_ptr<comms::ETHRecvComms<hytech_msgs::VCFData_s>> _vcf_eth_driver;
  std::unique_ptr<comms::ETHRecvComms<hytech_msgs::RemoteDriveCommand>> _remote_eth_driver;

  /* Sensoric OMS optical speed sensor */
  std::unique_ptr<comms::OMSComms> _oms_comms;

  /* Controllers */
  std::shared_ptr<control::LoadCellTorqueController> _mode1;
  std::shared_ptr<control::TeleopController> _mode2;

  /* Vectornav */
  std::unique_ptr<comms::VNDriver> _vn_driver;

#if !HOOTL_ENABLED
  /* Steering */
  std::unique_ptr<comms::KrakenComms> _kraken_comms;
#endif

  /* Cameras */
  std::vector<std::unique_ptr<comms::BlackflyComms>> _camera_drivers;

  /* Ouster Lidar */
  std::unique_ptr<comms::OusterComms> _ouster_driver;

  /* Driverless autonomy stack */
  core::Autonomy _autonomy;

#if HOOTL_ENABLED
  DrivingMode _driving_mode = DrivingMode::TELEOP;
#else
  DrivingMode _driving_mode = DrivingMode::REMOTE;
#endif
};
