#include <MCAPLogger.hpp>
#include <boost/asio.hpp>
#include <memory>
#include <EstimatorManager.hpp>
#include <MatlabModelAddHelper.hpp>

#include "ETHRecvComms.hpp"
#include "CANComms.hpp"
#include "SimComms.hpp"
#include "LoadCellTorqueController.hpp"
#include "VNComms.hpp"
#include "Autonomy.hpp"
#include "hytech_msgs.pb.h"

enum class DrivingMode { DRIVERED, DRIVERLESS };

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

  /* Controllers */
  std::shared_ptr<control::LoadCellTorqueController> _mode1;
  std::vector<std::shared_ptr<MatlabModel>> _gend_controllers;

  /* Vectornav */
  std::unique_ptr<comms::VNDriver> _vn_driver;

  /* Estimator Manager */
  std::shared_ptr<estimation::EstimatorManager> _estim_manager;

  /* Driverless autonomy stack */
  core::Autonomy _autonomy;
  DrivingMode _driving_mode = DrivingMode::DRIVERLESS;
};
