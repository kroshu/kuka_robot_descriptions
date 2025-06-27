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

#include <math.h>

#include <memory>
#include <rclcpp/parameter_value.hpp>

#include "kuka_gazebo/moveit_example.hpp"

int main(int argc, char * argv[])
{
  // Setup
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const example_node = std::make_shared<MoveitExample>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(example_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  example_node->initialize();
  example_node->addBreakPoint();

  // Go to correct position for the example
  auto init_trajectory = example_node->planToPosition(
    std::vector<double>{-0.7037, -0.8217, 1.3433, -0.8246, -0.0358, 0.0243});
  if (init_trajectory != nullptr)
  {
    example_node->moveGroupInterface()->execute(*init_trajectory);
  }

  example_node->addBreakPoint();

  auto planned_trajectory = example_node->planStraightPathAlongY("ompl", "");
  if (planned_trajectory != nullptr)
  {
    example_node->addBreakPoint();
    example_node->moveGroupInterface()->execute(*planned_trajectory);
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
