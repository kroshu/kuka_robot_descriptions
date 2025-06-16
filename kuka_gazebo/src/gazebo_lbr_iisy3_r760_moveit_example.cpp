#include <math.h>

#include <memory>
#include <rclcpp/parameter_value.hpp>

#include "kuka_gazebo/moveit_example.hpp"

int main(int argc, char * argv[]) {
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
  if (init_trajectory != nullptr) {
    example_node->moveGroupInterface()->execute(*init_trajectory);
  }

  example_node->addBreakPoint();

  auto planned_trajectory =
    example_node->planStraightPathAlongY("ompl", "");
  if (planned_trajectory != nullptr) {
    example_node->addBreakPoint();
    example_node->moveGroupInterface()->execute(*planned_trajectory);
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}