#include "rlc_executive/bt/bt_executor_node.hpp"

#include <behaviortree_cpp/basic_types.h>

#include <chrono>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rlc_executive/core/config_loader.hpp"
#include "rlc_executive/core/runtime_context.hpp"

namespace rlc_executive
{

BtExecutorNode::BtExecutorNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("rlc_executive", options)
{
  this->declare_parameter<std::string>("executor_config", "config/executor.yaml");
  this->declare_parameter<std::string>("profiles_config",
                                       "config/planning_profiles.yaml");
  this->declare_parameter<std::string>("plugins", "plugins/bt_plugins.txt");
  this->declare_parameter<std::string>("default_tree", "bt_trees/reactive_core.xml");
}

std::string BtExecutorNode::resolveSharePath(const std::string& subdir,
                                             const std::string& filename) const
{
  const std::string share = ament_index_cpp::get_package_share_directory("rlc_executive");
  return share + "/" + subdir + "/" + filename;
}

void BtExecutorNode::start()
{
  const auto exec_cfg_param = this->get_parameter("executor_config").as_string();
  const auto profiles_cfg_param = this->get_parameter("profiles_config").as_string();
  const auto plugins_param = this->get_parameter("plugins").as_string();
  const auto tree_param = this->get_parameter("default_tree").as_string();
  const auto share = ament_index_cpp::get_package_share_directory("rlc_executive");

  auto make_path = [&](const std::string& s) -> std::string {
    if (s.empty())
    {
      return s;
    }
    if (!s.empty() && s[0] == '/')
    {
      return s;
    }
    return share + "/" + s;
  };

  const std::string exec_cfg_path = make_path(exec_cfg_param);
  const std::string profiles_cfg_path = make_path(profiles_cfg_param);
  const std::string plugins_path = make_path(plugins_param);
  const std::string tree_xml_path = make_path(tree_param);

  RCLCPP_INFO(get_logger(), "Starting BT executive");
  RCLCPP_INFO(get_logger(), "executor_config: %s", exec_cfg_path.c_str());
  RCLCPP_INFO(get_logger(), "profiles_config: %s", profiles_cfg_path.c_str());
  RCLCPP_INFO(get_logger(), "plugins: %s", plugins_path.c_str());
  RCLCPP_INFO(get_logger(), "default_tree: %s", tree_xml_path.c_str());

  cfg_ = config::loadExecConfigFromFile(exec_cfg_path);
  config::loadPlanningProfilesInto(cfg_, profiles_cfg_path);

  RCLCPP_INFO(get_logger(), "tick_rate_hz: %.2f", cfg_.tick_rate_hz);

  ctx_ = std::make_shared<RuntimeContext>(*this, cfg_);

  tree_runner_.setTickOption(cfg_.tick_option);
  tree_runner_.setTickWhileRunningMaxBlock(cfg_.tick_while_running_max_block);

  tree_runner_.configure(plugins_path);
  tree_runner_.loadTreeFromXmlFile(tree_xml_path, ctx_);

  const auto period = std::chrono::duration<double>(1.0 / cfg_.tick_rate_hz);
  tick_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() { tick(); });
}

void BtExecutorNode::tick()
{
  const BT::NodeStatus status = tree_runner_.tick();
  RCLCPP_INFO(get_logger(), "BT status: %s", BT::toStr(status, false).c_str());
}

}  // namespace rlc_executive
