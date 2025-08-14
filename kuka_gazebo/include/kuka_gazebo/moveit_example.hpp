// Copyright 2025 KUKA Hungaria Kft.
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

#ifndef KUKA_GAZEBO__MOVEIT_EXAMPLE_HPP_
#define KUKA_GAZEBO__MOVEIT_EXAMPLE_HPP_

#include <math.h>

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"

class MoveitExample : public rclcpp::Node
{
public:
  MoveitExample() : rclcpp::Node("moveit_example") {}

  void initialize()
  {
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), PLANNING_GROUP);

    moveit_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(), "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface_->getRobotModel());

    moveit_visual_tools_->deleteAllMarkers();
    moveit_visual_tools_->loadRemoteControl();
    moveit_visual_tools_->trigger();

    planning_scene_diff_publisher_ =
      this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

    move_group_interface_->setMaxVelocityScalingFactor(0.1);
    move_group_interface_->setMaxAccelerationScalingFactor(0.1);
  }

  moveit_msgs::msg::RobotTrajectory::SharedPtr planToPosition(const std::vector<double> & joint_pos)
  {
    move_group_interface_->setJointValueTarget(joint_pos);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(LOGGER, "Sending planning request");
    if (!move_group_interface_->plan(plan))
    {
      RCLCPP_INFO(LOGGER, "Planning failed");
      return nullptr;
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Planning successful");
      return std::make_shared<moveit_msgs::msg::RobotTrajectory>(plan.trajectory_);
    }
  }

  moveit_msgs::msg::RobotTrajectory::SharedPtr planToPoint(
    const Eigen::Isometry3d & pose,
    const std::string & planning_pipeline = "pilz_industrial_motion_planner",
    const std::string & planner_id = "PTP")
  {
    // Create planning request using pilz industrial motion planner
    move_group_interface_->setPlanningPipelineId(planning_pipeline);
    move_group_interface_->setPlannerId(planner_id);
    move_group_interface_->setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(LOGGER, "Sending planning request");
    if (!move_group_interface_->plan(plan))
    {
      RCLCPP_INFO(LOGGER, "Planning failed");
      return nullptr;
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Planning successful");
      return std::make_shared<moveit_msgs::msg::RobotTrajectory>(plan.trajectory_);
    }
  }

  moveit_msgs::msg::RobotTrajectory::SharedPtr planStraightPathAlongY(
    const std::string & planning_pipeline = "pilz_industrial_motion_planner",
    const std::string & planner_id = "PTP")
  {
    // Create planning request using pilz industrial motion planner
    move_group_interface_->setPlanningPipelineId(planning_pipeline);
    move_group_interface_->setPlannerId(planner_id);

    auto current_pose = move_group_interface_->getCurrentPose();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose.pose);

    geometry_msgs::msg::Pose end_pose = current_pose.pose;
    end_pose.position.y -= 1.0;
    waypoints.push_back(end_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    RCLCPP_INFO(LOGGER, "Sending planning request");
    double fraction = move_group_interface_->computeCartesianPath(waypoints, 0.05, 0.0, trajectory);
    if (fraction == -1.0)
    {
      RCLCPP_INFO(LOGGER, "Planning failed");
      return nullptr;
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Planning successful");
      return std::make_shared<moveit_msgs::msg::RobotTrajectory>(trajectory);
    }
  }

  void addBreakPoint()
  {
    moveit_visual_tools_->trigger();
    moveit_visual_tools_->prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInterface()
  {
    return move_group_interface_;
  }

protected:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;
  const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_basic_plan");
  const std::string PLANNING_GROUP = "manipulator";
};

#endif  // KUKA_GAZEBO__MOVEIT_EXAMPLE_HPP_
