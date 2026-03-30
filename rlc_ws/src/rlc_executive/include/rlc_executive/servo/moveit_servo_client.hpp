#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/node.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <moveit_msgs/srv/servo_command_type.hpp>

#include "rlc_executive/core/exec_config.hpp"
#include "rlc_executive/core/types.hpp"

namespace rlc_executive
{

class MoveItServoClient final
{
public:
  using PauseSrv = std_srvs::srv::SetBool;
  using CmdTypeSrv = moveit_msgs::srv::ServoCommandType;

  enum class CommandType : std::int8_t
  {
    JOINT_JOG = CmdTypeSrv::Request::JOINT_JOG,
    TWIST = CmdTypeSrv::Request::TWIST,
    POSE = CmdTypeSrv::Request::POSE,
  };

  /**
   * @brief Creates clients for the MoveIt Servo control services and status topic.
   * @param[in] node ROS node used to create the clients, subscription, and timestamps.
   * @param[in] cfg Servo configuration with service and topic names.
   * @throws std::invalid_argument If any required service or topic name is empty.
   */
  MoveItServoClient(rclcpp::Node& node, const ServoConfig& cfg);

  ServoPauseResult
  setPaused(bool paused,
            std::chrono::milliseconds timeout = std::chrono::milliseconds(300));

  ServoCommandTypeResult
  setCommandType(CommandType type,
                 std::chrono::milliseconds timeout = std::chrono::milliseconds(300));

  std::optional<ServoStatusSnapshot> status() const;

private:
  void onStatus(const moveit_msgs::msg::ServoStatus::ConstSharedPtr& msg);

  rclcpp::Node* node_ = nullptr;
  const ServoConfig* cfg_ = nullptr;

  rclcpp::Client<PauseSrv>::SharedPtr pause_client_;
  rclcpp::Client<CmdTypeSrv>::SharedPtr cmd_type_client_;
  rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr status_sub_;

  mutable std::mutex status_mtx_;
  std::optional<moveit_msgs::msg::ServoStatus> last_status_;
  rclcpp::Time last_status_stamp_;
};

}  // namespace rlc_executive
