#pragma once

#include <mutex>
#include <optional>

#include <rclcpp/node.hpp>

#include "rlc_executive/core/exec_config.hpp"

namespace rlc_executive
{

class TeleoperationSession final
{
public:
  enum class DemoRequest
  {
    NONE,
    FINISH,
    ABORT
  };

  /**
   * @brief Creates a teleoperation session state holder.
   * @param[in] node ROS node used to timestamp incoming requests.
   */
  explicit TeleoperationSession(rclcpp::Node& node);

  DemoRequest takeRequest();
  DemoRequest peekRequest() const;

  void setRequest(DemoRequest request);

private:
  rclcpp::Node* node_ = nullptr;

  mutable std::mutex request_mtx_;
  DemoRequest request_ = DemoRequest::NONE;
  std::optional<rclcpp::Time> request_stamp_;
};

}  // namespace rlc_executive
