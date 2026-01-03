#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rlc_controller_cpp/controller_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<rlc_controller_cpp::ControllerNode>(options);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
