#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rlc_executive/bt/bt_executor_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rlc_executive::BtExecutorNode>();
  node->start();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}