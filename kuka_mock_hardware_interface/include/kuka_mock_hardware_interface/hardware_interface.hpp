// Copyright 2026 KUKA Hungaria Kft.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KUKA_MOCK_HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_
#define KUKA_MOCK_HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "mock_components/generic_system.hpp"

using hardware_interface::return_type;

namespace kuka_mock_hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

<<<<<<< HEAD
static constexpr size_t POSITION_INTERFACE_INDEX = 0;
static constexpr size_t VELOCITY_INTERFACE_INDEX = 1;
static constexpr size_t ACCELERATION_INTERFACE_INDEX = 2;

class HARDWARE_INTERFACE_PUBLIC KukaMockHardwareInterface
: public hardware_interface::SystemInterface
=======
class KukaMockHardwareInterface : public mock_components::GenericSystem
>>>>>>> f417daf (Clean up KUKA mock hardware implementation (#195))
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
    override;

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
    override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // Sunrise robot state structure
  struct RobotState
  {
    double tracking_performance_ = 1;
    // Enum values represented with doubles (as all interfaces must be pointers to doubles)
    double session_state_ = 0;
    double connection_quality_ = 0;
    double command_mode_ = 0;
    double safety_state_ = 0;
    double control_mode_ = 0;
    double operation_mode_ = 0;
    double drive_state_ = 0;
    double overlay_type_ = 0;
  };

  // Command interfaces
  double control_mode_ = 0;  // default to undefined
  double receive_multiplier_ = 1;
  double send_period_ms_ = 10;
  double cycle_time_ms_ = 4;

  // State interfaces
  double drive_state_ = 0.0;
  double server_state_ = 0.0;
  double control_mode_state_ = 0.0;
  double cycle_time_state_ = 0.0;
  double drives_powered_ = 0.0;
  double emergency_stop_ = 0.0;
  double guard_stop_ = 0.0;
  double in_motion_ = 0.0;
  double motion_possible_ = 0.0;
  double operation_mode_ = 0.0;
  double robot_stopped_ = 1.0;
  RobotState robot_state_;

  // KUKA-specific parameters
  std::chrono::nanoseconds cycle_time_nano_;
  int roundtrip_time_micro_;
  bool init_clock_ = true;
  std::chrono::steady_clock::time_point next_iteration_time_;
};

}  // namespace kuka_mock_hardware_interface

#endif  // KUKA_MOCK_HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_
