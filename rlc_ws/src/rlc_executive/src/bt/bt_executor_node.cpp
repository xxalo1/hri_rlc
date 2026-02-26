#include "rlc_executive/bt/bt_executor_node.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <chrono>
#include <cstdint>
#include <memory>
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
  this->declare_parameter<bool>("enable_groot2", false);
  this->declare_parameter<int>("groot2_port", 1667);
}

BtExecutorNode::~BtExecutorNode() = default;

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

  const bool enable_groot2 = this->get_parameter("enable_groot2").as_bool();
  const std::int64_t groot2_port_param = this->get_parameter("groot2_port").as_int();

  if (enable_groot2)
  {
    if (groot2_port_param <= 0 || groot2_port_param > 65535)
    {
      RCLCPP_ERROR(get_logger(),
                   "enable_groot2=true but groot2_port=%ld is invalid; "
                   "disabling Groot2Publisher",
                   groot2_port_param);
    }
    else
    {
      const unsigned port = static_cast<unsigned>(groot2_port_param);
      try
      {
        groot2_publisher_ =
            std::make_unique<BT::Groot2Publisher>(tree_runner_.tree(), port);
        RCLCPP_INFO(get_logger(), "Groot2Publisher listening on tcp://0.0.0.0:%u", port);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(get_logger(),
                     "Failed to start Groot2Publisher on port %u: %s (continuing "
                     "without Groot2)",
                     port, e.what());
      }
    }
  }

  const auto period = std::chrono::duration<double>(1.0 / cfg_.tick_rate_hz);
  tick_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() { tick(); });
}

void BtExecutorNode::tick()
{
  if (completed_)
  {
    return;
  }

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  try
  {
    status = tree_runner_.tick();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "BT tick threw exception: %s", e.what());
    onTreeCompleted(BT::NodeStatus::FAILURE);
    return;
  }

  if (status != BT::NodeStatus::RUNNING)
  {
    onTreeCompleted(status);
  }
}

void BtExecutorNode::onTreeCompleted(BT::NodeStatus status)
{
  if (completed_)
  {
    return;
  }
  completed_ = true;

  if (tick_timer_)
  {
    tick_timer_->cancel();  // prevent further ticks ASAP
  }

  try
  {
    tree_runner_.haltTree();
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(get_logger(), "haltTree threw: %s", e.what());
  }

  RCLCPP_INFO(get_logger(), "BT completed with status: %s",
              BT::toStr(status, false).c_str());

  rclcpp::shutdown();
}
}  // namespace rlc_executive
