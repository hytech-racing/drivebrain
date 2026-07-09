#include "ETHRecvComms.hpp"
#include "CANComms.hpp"
#include "SimComms.hpp"
#include <MCAPLogger.hpp>
#include <boost/asio.hpp>
#include <memory>
#include "LoadCellTorqueController.hpp"
#include "VNComms.hpp"

#include "InveterInterface.hpp"
#include "LidarInterface.hpp"
#include "SteeringInterface.hpp"
#include "SimStateReceiver.hpp"
#include "hytech_msgs.pb.h"
#include <EstimatorManager.hpp>

class DrivebrainApp {
public:
  DrivebrainApp(const std::string& json_params_path, const std::string& dbc_path);
  ~DrivebrainApp();

  DrivebrainApp& operator=(DrivebrainApp&&) = delete;
  DrivebrainApp(DrivebrainApp&&) = delete;

  void run();

private:
  void _loop();

private:
  boost::asio::io_context _io_context;

  std::thread _loop_thread;
  std::thread _io_context_thread;

  const std::string _json_params_path;
  const std::string _dbc_path;

  std::unique_ptr<InverterInterface> _inverter;
  std::unique_ptr<SteeringInterface> _steering;
  std::unique_ptr<LidarInterface> _lidar;
  std::unique_ptr<SimStateReceiver> _sim_state_receiver;

  std::unique_ptr<comms::ETHRecvComms<hytech_msgs::ACUAllData>> _acu_eth_driver;
  std::unique_ptr<comms::ETHRecvComms<hytech_msgs::ACUCoreData>> _acu_core_eth_driver;
  std::unique_ptr<comms::ETHRecvComms<hytech_msgs::VCRData_s>> _vcr_eth_driver;
  std::unique_ptr<comms::ETHRecvComms<hytech_msgs::VCFData_s>> _vcf_eth_driver;

  std::shared_ptr<control::LoadCellTorqueController> _mode1;

  std::unique_ptr<comms::VNDriver> _vn_driver;

  std::shared_ptr<estimation::EstimatorManager> _estim_manager;
};
