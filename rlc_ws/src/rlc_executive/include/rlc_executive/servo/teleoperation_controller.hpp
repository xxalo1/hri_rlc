#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>

#include <rclcpp/node.hpp>

#include "rlc_executive/core/exec_config.hpp"
#include "rlc_executive/core/types.hpp"
#include "rlc_executive/servo/moveit_servo_client.hpp"
#include "rlc_executive/servo/teleoperation_frontend.hpp"
#include "rlc_executive/servo/teleoperation_session.hpp"

namespace rlc_executive
{

/**
 * @brief Owns the Servo teleoperation stack used by runtime and BT leaves.
 *
 * @details
 * This controller owns the MoveIt Servo client, the teleoperation request session,
 * and the selected operator-input frontend. Callers should interact with teleoperation
 * through this class instead of wiring lower-level teleop objects separately.
 */
class TeleoperationController final
{
public:
  /**
   * @brief Constructs the complete Servo teleoperation stack.
   * @param[in] node ROS node used to create teleoperation ROS interfaces.
   * @param[in] teleop_cfg Top-level teleoperation configuration, including the selected
   * Servo client and frontend settings.
   * @throws std::invalid_argument If any owned dependency rejects the provided configuration.
   */
  TeleoperationController(rclcpp::Node& node, const TeleoperationConfig& teleop_cfg);

  /**
   * @brief Activates or deactivates Servo teleoperation as one operation.
   * @param[in] active `true` to unpause Servo and enable the active frontend, `false` to
   * disable the active frontend and pause Servo.
   * @param[in] timeout Maximum wait time [ms] used by the Servo pause service call.
   * @return Result of the Servo pause service request issued by this controller.
   */
  ServoPauseResult
  setActive(bool active,
            std::chrono::milliseconds timeout = std::chrono::milliseconds(300));

  /// @brief Returns whether the controller currently considers teleoperation active.
  bool isActive() const noexcept
  { return active_.load(std::memory_order_relaxed); }

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
   * @brief Returns the most recent Servo status sample, if one has been received.
   * @return Latest Servo status snapshot, or `std::nullopt` if no status has been observed.
   */
  std::optional<ServoStatusSnapshot> status() const;

private:
  TeleoperationConfig cfg_;
  MoveItServoClient servo_client_;
  TeleoperationSession session_;
  std::unique_ptr<TeleoperationFrontend> frontend_;
  std::atomic_bool active_{ false };
};

}  // namespace rlc_executive
