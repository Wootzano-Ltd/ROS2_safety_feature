#include "safety_feature/safety_feature.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("safety_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  SafetyFeature safety_feature(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}