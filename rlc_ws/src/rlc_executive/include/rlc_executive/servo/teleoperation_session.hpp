#pragma once

#include <mutex>
#include <optional>

#include <rclcpp/node.hpp>

#include "rlc_executive/core/types.hpp"

namespace rlc_executive
{

/**
 * @brief Stores the latest operator request produced by the active teleoperation frontend.
 */
class TeleoperationSession final
{
public:
  /**
   * @brief Creates a teleoperation request latch.
   * @param[in] node ROS node used to timestamp incoming requests.
   */
  explicit TeleoperationSession(rclcpp::Node& node);

  /**
   * @brief Returns and clears the currently latched operator request.
   * @return Pending operator request, or `DemoRequest::NONE` if no request is pending.
   */
  DemoRequest takeRequest();

  /**
   * @brief Returns the currently latched operator request without clearing it.
   * @return Pending operator request, or `DemoRequest::NONE` if no request is pending.
   */
  DemoRequest peekRequest() const;

  /**
   * @brief Stores one operator request for later consumption.
   * @param[in] request Pending operator request to latch.
   */
  void setRequest(DemoRequest request);

private:
  rclcpp::Node* node_ = nullptr;

  mutable std::mutex request_mtx_;
  DemoRequest request_ = DemoRequest::NONE;
  std::optional<rclcpp::Time> request_stamp_;
};

}  // namespace rlc_executive
