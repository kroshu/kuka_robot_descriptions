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

#include <memory>

#include "rcutils/logging_macros.h"

namespace kuka_mock_hardware_interface
{
CallbackReturn KukaMockHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
<<<<<<< HEAD
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  auto populate_non_standard_interfaces =
    [this](auto interface_list, auto & non_standard_interfaces)
  {
    for (const auto & interface : interface_list)
    {
      // add to list if non-standard interface
      if (
        std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name) ==
        standard_interfaces_.end())
      {
        if (
          std::find(
            non_standard_interfaces.begin(), non_standard_interfaces.end(), interface.name) ==
          non_standard_interfaces.end())
        {
          non_standard_interfaces.emplace_back(interface.name);
        }
      }
    }
  };

  // check if to create mock command interface for sensor
  auto it = info_.hardware_parameters.find("mock_sensor_commands");
  if (it != info_.hardware_parameters.end())
  {
    use_mock_sensor_command_interfaces_ = hardware_interface::parse_bool(it->second);
  }
  else
  {
    // check if fake_sensor_commands was set instead and issue warning.
    it = info_.hardware_parameters.find("fake_sensor_commands");
    if (it != info_.hardware_parameters.end())
    {
      use_mock_sensor_command_interfaces_ = hardware_interface::parse_bool(it->second);
      RCUTILS_LOG_WARN_NAMED(
        "mock_generic_system",
        "Parameter 'fake_sensor_commands' has been deprecated from usage. Use"
        "'mock_sensor_commands' instead.");
    }
    else
    {
      use_mock_sensor_command_interfaces_ = false;
    }
  }

  // check if to create mock command interface for gpio
  it = info_.hardware_parameters.find("mock_gpio_commands");
  if (it != info_.hardware_parameters.end())
  {
    use_mock_gpio_command_interfaces_ = hardware_interface::parse_bool(it->second);
  }
  else
  {
    // check if fake_gpio_commands was set instead and issue warning
    it = info_.hardware_parameters.find("fake_gpio_commands");
    if (it != info_.hardware_parameters.end())
    {
      use_mock_gpio_command_interfaces_ = hardware_interface::parse_bool(it->second);
      RCUTILS_LOG_WARN_NAMED(
        "mock_generic_system",
        "Parameter 'fake_gpio_commands' has been deprecated from usage. Use"
        "'mock_gpio_commands' instead.");
    }
    else
    {
      use_mock_gpio_command_interfaces_ = false;
    }
  }

  // check if there is parameter that disables commands
  // this way we simulate disconnected driver
  it = info_.hardware_parameters.find("disable_commands");
  if (it != info.hardware_parameters.end())
  {
    command_propagation_disabled_ = hardware_interface::parse_bool(it->second);
  }
  else
  {
    command_propagation_disabled_ = false;
  }

  // check if there is parameter that enables dynamic calculation
  it = info_.hardware_parameters.find("calculate_dynamics");
  if (it != info.hardware_parameters.end())
  {
    calculate_dynamics_ = hardware_interface::parse_bool(it->second);
  }
  else
  {
    calculate_dynamics_ = false;
  }

  // process parameters about state following
  position_state_following_offset_ = 0.0;
  custom_interface_with_following_offset_ = "";

  it = info_.hardware_parameters.find("position_state_following_offset");
  if (it != info_.hardware_parameters.end())
  {
    position_state_following_offset_ = std::stod(it->second);
    it = info_.hardware_parameters.find("custom_interface_with_following_offset");
    if (it != info_.hardware_parameters.end())
    {
      custom_interface_with_following_offset_ = it->second;
    }
  }

  // Parse KUKA-specific parameters
  it = info_.hardware_parameters.find("cycle_time_ms");
  if (it != info.hardware_parameters.end())
=======
  auto ret = mock_components::GenericSystem::on_init(params);
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Parse KUKA-specific parameters
  auto info = get_hardware_info();
  auto it = info.hardware_parameters.find("cycle_time_ms");
  if (it != info.hardware_parameters.end() && std::stoi(it->second) > 0)
>>>>>>> f417daf (Clean up KUKA mock hardware implementation (#195))
  {
    cycle_time_nano_ = std::chrono::nanoseconds(std::stoi(it->second) * 1'000'000);
    RCUTILS_LOG_INFO_NAMED(
      "mock_generic_system", "Using configured cycle time of %d [ms]", std::stoi(it->second));
  }
  else
  {
    cycle_time_nano_ = std::chrono::nanoseconds(4'000'000);  // Default to 4 ms
    RCUTILS_LOG_INFO_NAMED("mock_generic_system", "Using default cycle time of 4 [ms]");
  }

<<<<<<< HEAD
  it = info_.hardware_parameters.find("roundtrip_time_micro");
  if (it != info.hardware_parameters.end())
=======
  it = info.hardware_parameters.find("roundtrip_time_micro");
  if (it != info.hardware_parameters.end() && std::stoi(it->second) > 0)
>>>>>>> f417daf (Clean up KUKA mock hardware implementation (#195))
  {
    roundtrip_time_micro_ = std::stoi(it->second);
    RCUTILS_LOG_INFO_NAMED(
      "mock_generic_system", "Setting allowed roundtrip time to %d [us]", std::stoi(it->second));
  }
  else
  {
    roundtrip_time_micro_ = 0;  // Default to no timeout checking
<<<<<<< HEAD
  }

  // its extremlly improbably that std::distance results int this value - therefore default
  index_custom_interface_with_following_offset_ = std::numeric_limits<size_t>::max();

  // Initialize storage for standard interfaces
  initialize_storage_vectors(joint_commands_, joint_states_, standard_interfaces_, info_.joints);
  // set all values without initial values to 0
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    for (auto j = 0u; j < standard_interfaces_.size(); j++)
    {
      if (std::isnan(joint_states_[j][i]))
      {
        joint_states_[j][i] = 0.0;
      }
    }
  }

  // Search for mimic joints
  for (auto i = 0u; i < info_.joints.size(); ++i)
  {
    const auto & joint = info_.joints.at(i);
    if (joint.parameters.find("mimic") != joint.parameters.cend())
    {
      const auto mimicked_joint_it = std::find_if(
        info_.joints.begin(), info_.joints.end(),
        [&mimicked_joint =
           joint.parameters.at("mimic")](const hardware_interface::ComponentInfo & joint_info)
        { return joint_info.name == mimicked_joint; });
      if (mimicked_joint_it == info_.joints.cend())
      {
        throw std::runtime_error(
          std::string("Mimicked joint '") + joint.parameters.at("mimic") + "' not found");
      }
      MimicJoint mimic_joint;
      mimic_joint.joint_index = i;
      mimic_joint.mimicked_joint_index = std::distance(info_.joints.begin(), mimicked_joint_it);
      auto param_it = joint.parameters.find("multiplier");
      if (param_it != joint.parameters.end())
      {
        mimic_joint.multiplier = std::stod(joint.parameters.at("multiplier"));
      }
      mimic_joints_.push_back(mimic_joint);
    }
  }

  // search for non-standard joint interfaces
  for (const auto & joint : info_.joints)
  {
    // populate non-standard command interfaces to other_interfaces_
    populate_non_standard_interfaces(joint.command_interfaces, other_interfaces_);

    // populate non-standard state interfaces to other_interfaces_
    populate_non_standard_interfaces(joint.state_interfaces, other_interfaces_);
  }

  // Initialize storage for non-standard interfaces
  initialize_storage_vectors(other_commands_, other_states_, other_interfaces_, info_.joints);

  // when following offset is used on custom interface then find its index
  if (!custom_interface_with_following_offset_.empty())
  {
    auto if_it = std::find(
      other_interfaces_.begin(), other_interfaces_.end(), custom_interface_with_following_offset_);
    if (if_it != other_interfaces_.end())
    {
      index_custom_interface_with_following_offset_ =
        std::distance(other_interfaces_.begin(), if_it);
      RCUTILS_LOG_INFO_NAMED(
        "mock_generic_system", "Custom interface with following offset '%s' found at index: %zu.",
        custom_interface_with_following_offset_.c_str(),
        index_custom_interface_with_following_offset_);
    }
    else
    {
      RCUTILS_LOG_WARN_NAMED(
        "mock_generic_system",
        "Custom interface with following offset '%s' does not exist. Offset will not be applied",
        custom_interface_with_following_offset_.c_str());
    }
  }

  for (const auto & sensor : info_.sensors)
  {
    for (const auto & interface : sensor.state_interfaces)
    {
      if (
        std::find(sensor_interfaces_.begin(), sensor_interfaces_.end(), interface.name) ==
        sensor_interfaces_.end())
      {
        sensor_interfaces_.emplace_back(interface.name);
      }
    }
  }
  initialize_storage_vectors(
    sensor_mock_commands_, sensor_states_, sensor_interfaces_, info_.sensors);

  // search for gpio interfaces
  for (const auto & gpio : info_.gpios)
  {
    // populate non-standard command interfaces to gpio_interfaces_
    populate_non_standard_interfaces(gpio.command_interfaces, gpio_interfaces_);

    // populate non-standard state interfaces to gpio_interfaces_
    populate_non_standard_interfaces(gpio.state_interfaces, gpio_interfaces_);
  }

  // Mock gpio command interfaces
  if (use_mock_gpio_command_interfaces_)
  {
    initialize_storage_vectors(gpio_mock_commands_, gpio_states_, gpio_interfaces_, info_.gpios);
  }
  // Real gpio command interfaces
  else
  {
    initialize_storage_vectors(gpio_commands_, gpio_states_, gpio_interfaces_, info_.gpios);
=======
    RCUTILS_LOG_INFO_NAMED("mock_generic_system", "Roundtrip time will not be monitored");
>>>>>>> f417daf (Clean up KUKA mock hardware implementation (#195))
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaMockHardwareInterface::on_configure(const rclcpp_lifecycle::State & state)
{
  auto ret = mock_components::GenericSystem::on_configure(state);
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }
  init_clock_ = true;
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
KukaMockHardwareInterface::on_export_state_interfaces()
{
  auto state_interfaces = mock_components::GenericSystem::on_export_state_interfaces();

<<<<<<< HEAD
  // Joints' state interfaces
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    const auto & joint = info_.joints[i];
    for (const auto & interface : joint.state_interfaces)
    {
      // Add interface: if not in the standard list then use "other" interface list
      if (!get_interface(
            joint.name, standard_interfaces_, interface.name, i, joint_states_, state_interfaces))
      {
        if (!get_interface(
              joint.name, other_interfaces_, interface.name, i, other_states_, state_interfaces))
        {
          throw std::runtime_error(
            "Interface is not found in the standard nor other list. "
            "This should never happen!");
        }
      }
    }
  }

  // Sensor state interfaces
  if (!populate_interfaces(
        info_.sensors, sensor_interfaces_, sensor_states_, state_interfaces, true))
  {
    throw std::runtime_error(
      "Interface is not found in the standard nor other list. This should never happen!");
  };

  // GPIO state interfaces
  if (!populate_interfaces(info_.gpios, gpio_interfaces_, gpio_states_, state_interfaces, true))
  {
    throw std::runtime_error("Interface is not found in the gpio list. This should never happen!");
  }

  state_interfaces.emplace_back(
=======
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
>>>>>>> f417daf (Clean up KUKA mock hardware implementation (#195))
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::SESSION_STATE,
    &robot_state_.session_state_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::CONNECTION_QUALITY,
    &robot_state_.connection_quality_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::SAFETY_STATE,
    &robot_state_.safety_state_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::COMMAND_MODE,
    &robot_state_.command_mode_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::CONTROL_MODE,
    &robot_state_.control_mode_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::OPERATION_MODE,
    &robot_state_.operation_mode_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::DRIVE_STATE,
    &robot_state_.drive_state_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::OVERLAY_TYPE,
    &robot_state_.overlay_type_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::FRI_STATE_PREFIX, hardware_interface::TRACKING_PERFORMANCE,
    &robot_state_.tracking_performance_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::STATE_PREFIX, hardware_interface::SERVER_STATE, &server_state_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::STATE_PREFIX, hardware_interface::CONTROL_MODE, &control_mode_state_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::STATE_PREFIX, hardware_interface::CYCLE_TIME, &cycle_time_state_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::STATE_PREFIX, hardware_interface::DRIVES_POWERED, &drives_powered_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::STATE_PREFIX, hardware_interface::EMERGENCY_STOP, &emergency_stop_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::STATE_PREFIX, hardware_interface::GUARD_STOP, &guard_stop_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::STATE_PREFIX, hardware_interface::IN_MOTION, &in_motion_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::STATE_PREFIX, hardware_interface::MOTION_POSSIBLE, &motion_possible_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::STATE_PREFIX, hardware_interface::OPERATION_MODE, &operation_mode_));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    hardware_interface::STATE_PREFIX, hardware_interface::ROBOT_STOPPED, &robot_stopped_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
KukaMockHardwareInterface::on_export_command_interfaces()
{
  auto command_interfaces = mock_components::GenericSystem::on_export_command_interfaces();

<<<<<<< HEAD
  // Joints' state interfaces
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint = info_.joints[i];
    for (const auto & interface : joint.command_interfaces)
    {
      // Add interface: if not in the standard list than use "other" interface list
      if (!get_interface(
            joint.name, standard_interfaces_, interface.name, i, joint_commands_,
            command_interfaces))
      {
        if (!get_interface(
              joint.name, other_interfaces_, interface.name, i, other_commands_,
              command_interfaces))
        {
          throw std::runtime_error(
            "Interface is not found in the standard nor other list. "
            "This should never happen!");
        }
      }
    }
  }
  // Set position control mode per default
  joint_control_mode_.resize(info_.joints.size(), POSITION_INTERFACE_INDEX);

  // Mock sensor command interfaces
  if (use_mock_sensor_command_interfaces_)
  {
    if (!populate_interfaces(
          info_.sensors, sensor_interfaces_, sensor_mock_commands_, command_interfaces, true))
    {
      throw std::runtime_error(
        "Interface is not found in the standard nor other list. This should never happen!");
    }
  }

  // Mock gpio command interfaces (consider all state interfaces for command interfaces)
  if (use_mock_gpio_command_interfaces_)
  {
    if (!populate_interfaces(
          info_.gpios, gpio_interfaces_, gpio_mock_commands_, command_interfaces, true))
    {
      throw std::runtime_error(
        "Interface is not found in the gpio list. This should never happen!");
    }
  }
  // GPIO command interfaces (real command interfaces)
  else
  {
    if (!populate_interfaces(
          info_.gpios, gpio_interfaces_, gpio_commands_, command_interfaces, false))
    {
      throw std::runtime_error(
        "Interface is not found in the gpio list. This should never happen!");
    }
  }

  command_interfaces.emplace_back(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CONTROL_MODE, &control_mode_);
  command_interfaces.emplace_back(
=======
  command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CONTROL_MODE, &control_mode_));
  command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
>>>>>>> f417daf (Clean up KUKA mock hardware implementation (#195))
    hardware_interface::CONFIG_PREFIX, hardware_interface::RECEIVE_MULTIPLIER,
    &receive_multiplier_));
  command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
    hardware_interface::CONFIG_PREFIX, hardware_interface::SEND_PERIOD, &send_period_ms_));
  command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
    hardware_interface::CONFIG_PREFIX, hardware_interface::DRIVE_STATE, &drive_state_));
  command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
    hardware_interface::CONFIG_PREFIX, hardware_interface::CYCLE_TIME, &cycle_time_ms_));

  return command_interfaces;
}

<<<<<<< HEAD
return_type KukaMockHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  if (!calculate_dynamics_)
  {
    return ret_val;
  }

  const size_t FOUND_ONCE_FLAG = 1000000;

  std::vector<size_t> joint_found_in_x_requests_;
  joint_found_in_x_requests_.resize(info_.joints.size(), 0);

  for (const auto & key : start_interfaces)
  {
    // check if interface is joint
    auto joint_it_found = std::find_if(
      info_.joints.begin(), info_.joints.end(),
      [key](const auto & joint) { return (key.find(joint.name) != std::string::npos); });

    if (joint_it_found != info_.joints.end())
    {
      const size_t joint_index = std::distance(info_.joints.begin(), joint_it_found);
      if (joint_found_in_x_requests_[joint_index] == 0)
      {
        joint_found_in_x_requests_[joint_index] = FOUND_ONCE_FLAG;
      }

      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_POSITION)
      {
        joint_found_in_x_requests_[joint_index] += 1;
      }
      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        if (!calculate_dynamics_)
        {
          RCUTILS_LOG_WARN_NAMED(
            "mock_generic_system",
            "Requested velocity mode for joint '%s' without dynamics calculation enabled - this "
            "might lead to wrong feedback and unexpected behavior.",
            info_.joints[joint_index].name.c_str());
        }
        joint_found_in_x_requests_[joint_index] += 1;
      }
      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_ACCELERATION)
      {
        if (!calculate_dynamics_)
        {
          RCUTILS_LOG_WARN_NAMED(
            "mock_generic_system",
            "Requested acceleration mode for joint '%s' without dynamics calculation enabled - "
            "this might lead to wrong feedback and unexpected behavior.",
            info_.joints[joint_index].name.c_str());
        }
        joint_found_in_x_requests_[joint_index] += 1;
      }
    }
    else
    {
      RCUTILS_LOG_DEBUG_NAMED(
        "mock_generic_system", "Got interface '%s' that is not joint - nothing to do!",
        key.c_str());
    }
  }

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // There has to always be at least one control mode from the above three set
    if (joint_found_in_x_requests_[i] == FOUND_ONCE_FLAG)
    {
      RCUTILS_LOG_ERROR_NAMED(
        "mock_generic_system", "Joint '%s' has to have '%s', '%s', or '%s' interface!",
        info_.joints[i].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      ret_val = hardware_interface::return_type::ERROR;
    }

    // Currently we don't support multiple interface request
    if (joint_found_in_x_requests_[i] > (FOUND_ONCE_FLAG + 1))
    {
      RCUTILS_LOG_ERROR_NAMED(
        "mock_generic_system",
        "Got multiple (%zu) starting interfaces for joint '%s' - this is not "
        "supported!",
        joint_found_in_x_requests_[i] - FOUND_ONCE_FLAG, info_.joints[i].name.c_str());
      ret_val = hardware_interface::return_type::ERROR;
    }
  }

  return ret_val;
}

return_type KukaMockHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  if (!calculate_dynamics_)
  {
    return hardware_interface::return_type::OK;
  }

  for (const auto & key : start_interfaces)
  {
    // check if interface is joint
    auto joint_it_found = std::find_if(
      info_.joints.begin(), info_.joints.end(),
      [key](const auto & joint) { return (key.find(joint.name) != std::string::npos); });

    if (joint_it_found != info_.joints.end())
    {
      const size_t joint_index = std::distance(info_.joints.begin(), joint_it_found);

      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_POSITION)
      {
        joint_control_mode_[joint_index] = POSITION_INTERFACE_INDEX;
      }
      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        joint_control_mode_[joint_index] = VELOCITY_INTERFACE_INDEX;
      }
      if (key == info_.joints[joint_index].name + "/" + hardware_interface::HW_IF_ACCELERATION)
      {
        joint_control_mode_[joint_index] = ACCELERATION_INTERFACE_INDEX;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

=======
>>>>>>> f417daf (Clean up KUKA mock hardware implementation (#195))
return_type KukaMockHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto ret = mock_components::GenericSystem::read(time, period);
  if (ret != return_type::OK)
  {
    return ret;
  }

  if (init_clock_)
  {
    // Initialize from a monotonic clock to avoid wall-clock jumps
    next_iteration_time_ = std::chrono::steady_clock::now();
    init_clock_ = false;
  }

  next_iteration_time_ += std::chrono::nanoseconds(cycle_time_nano_);
  std::this_thread::sleep_until(next_iteration_time_);

  return return_type::OK;
}

return_type KukaMockHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto ret = mock_components::GenericSystem::write(time, period);
  if (ret != return_type::OK)
  {
    return ret;
  }

  if (roundtrip_time_micro_ != 0)
  {
    const auto now = std::chrono::steady_clock::now();
    const auto allowed_time =
      next_iteration_time_ + std::chrono::microseconds(roundtrip_time_micro_);

    if (now > allowed_time)
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
