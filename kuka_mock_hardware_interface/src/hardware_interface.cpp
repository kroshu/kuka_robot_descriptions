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

#include "kuka_mock_hardware_interface/hardware_interface.hpp"
#include "kuka_mock_hardware_interface/hardware_interface_types.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"

namespace kuka_mock_hardware_interface
{
CallbackReturn KukaMockHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (mock_components::GenericSystem::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
 
  // Parse KUKA-specific parameters
  auto info = get_hardware_info();
  auto it = info.hardware_parameters.find("cycle_time_ms");
  if (it != info.hardware_parameters.end())
  {
    cycle_time_nano_ = std::chrono::nanoseconds(std::stoi(it->second) * 1'000'000);
  }
  else
  {
    cycle_time_nano_ = std::chrono::nanoseconds(4'000'000);  // Default to 4 ms
  }

  it = info.hardware_parameters.find("roundtrip_time_micro");
  if (it != info.hardware_parameters.end())
  {
    roundtrip_time_micro_ = std::stod(it->second);
  }
  else
  {
    roundtrip_time_micro_ = 0;  // Default to no timeout checking
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaMockHardwareInterface::on_configure(const rclcpp_lifecycle::State& state)
{
  if (mock_components::GenericSystem::on_configure(state) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  init_clock_ = true;
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KukaMockHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces = mock_components::GenericSystem::export_state_interfaces();

  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::SESSION_STATE,
    &robot_state_.session_state_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::CONNECTION_QUALITY,
    &robot_state_.connection_quality_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::SAFETY_STATE,
    &robot_state_.safety_state_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::COMMAND_MODE,
    &robot_state_.command_mode_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::CONTROL_MODE,
    &robot_state_.control_mode_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::OPERATION_MODE,
    &robot_state_.operation_mode_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::DRIVE_STATE,
    &robot_state_.drive_state_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::OVERLAY_TYPE,
    &robot_state_.overlay_type_);
  state_interfaces.emplace_back(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::TRACKING_PERFORMANCE,
    &robot_state_.tracking_performance_);
  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::SERVER_STATE, &server_state_);
  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::CONTROL_MODE, &control_mode_state_);
  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::CYCLE_TIME, &cycle_time_state_);
  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::DRIVES_POWERED, &drives_powered_);
  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::EMERGENCY_STOP, &emergency_stop_);
  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::GUARD_STOP, &guard_stop_);
  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::IN_MOTION, &in_motion_);
  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::MOTION_POSSIBLE, &motion_possible_);
  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::OPERATION_MODE, &operation_mode_);
  state_interfaces.emplace_back(
    hardware_interface::STATE_PREFIX, hardware_interface::ROBOT_STOPPED, &robot_stopped_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KukaMockHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces = mock_components::GenericSystem::export_command_interfaces();

  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CONTROL_MODE, &control_mode_);
  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::RECEIVE_MULTIPLIER,
    &receive_multiplier_);
  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::SEND_PERIOD, &send_period_ms_);
  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::DRIVE_STATE, &drive_state_);
  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CYCLE_TIME, &cycle_time_ms_);

  return command_interfaces;
}


return_type KukaMockHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{

  auto ret = mock_components::GenericSystem::read(time, period);
  if (ret != return_type::OK) return ret;

  if (init_clock_)
  {
    next_iteration_time_ =
      std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>(
        std::chrono::nanoseconds(time.nanoseconds()));
    init_clock_ = false;
  }

  next_iteration_time_ += std::chrono::nanoseconds(cycle_time_nano_);
  std::this_thread::sleep_until(next_iteration_time_);

  return return_type::OK;
}

return_type KukaMockHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{

  auto ret = mock_components::GenericSystem::write(time, period);
  if (ret != return_type::OK) return ret;

  if (roundtrip_time_micro_ != 0)
  {
    if (
      time.nanoseconds() >
      next_iteration_time_.time_since_epoch().count() + roundtrip_time_micro_ * 1000)
    {
      RCUTILS_LOG_WARN_NAMED("mock_generic_system", "Cycle exceeded allowed round-trip time");
    }
  }
  return return_type::OK;
}

}  // namespace kuka_mock_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  kuka_mock_hardware_interface::KukaMockHardwareInterface, hardware_interface::SystemInterface)
