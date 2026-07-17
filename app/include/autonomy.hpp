#pragma once

#include <atomic>
#include <chrono>
#include <thread>

#include <StateTracker.hpp>
#include <GraphSLAM.hpp>

namespace core {

/**
 * The autonomy stack. 
 * Runs the driverless perception, mapping and planning stack on its own thread
 */
class Autonomy {
public:
  ~Autonomy();

  /**
   * Spawns the autonomy stack thread
   */
  void start();

  /**
   * Stops and joins the autonomy stack thread
   */
  void stop();

  /**
   * Produces the driverless command by tracking the latest planned path.
   * Called by the control loop at control rate.
   *
   * @param vehicle_state the latest vehicle state
   * @return the controller output to send
   */
  ControllerOutput command(const VehicleState& vehicle_state);

  /**
   * Whether the stack is producing usable output. The control loop gates driverless
   * commands on this the same way driver mode gates on the driver input timestamps.
   *
   * @return whether the car can be commanded from the autonomy stack
   */
  bool is_valid();

private:
  // filter -> classify -> SLAM -> plan
  void _run();

  std::thread _thread;
  std::atomic<bool> _running{false};

  slam::GraphSlam _slam;

  // Rate of OS1 LiDAR
  static constexpr std::chrono::milliseconds _period{100}; 
};

}
