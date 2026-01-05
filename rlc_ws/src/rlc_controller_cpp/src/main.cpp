#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "rlc_controller_cpp/controller_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<rlc_controller_cpp::ControllerNode>(options);

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);

  rclcpp::shutdown();
  return 0;
}
