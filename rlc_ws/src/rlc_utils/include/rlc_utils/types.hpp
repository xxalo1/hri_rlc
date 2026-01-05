#pragma once

/**
 * @file types.hpp
 * @brief Preallocated data containers for fast ROS message conversion.
 */

#include <Eigen/Core>
#include <cstddef>
#include <string>
#include <stdexcept>
#include <vector>

namespace rlc_utils::types {

/**
 * @brief Preallocated joint state container matching `sensor_msgs::msg::JointState`.
 *
 * @details
 * Sizes:
 * - `name`: length = `size()` (may be left unset by converters).
 * - `position`: Joint positions [rad], size = `size()`.
 * - `velocity`: Joint velocities [rad/s], size = `size()`.
 * - `effort`: Joint effort/torque [N·m], size = `size()`.
 *
 * Timestamp:
 * - `stamp_sec`: Message stamp [s], typically derived from `header.stamp`.
 */
struct JointStateMsgData {
  std::vector<std::string> name;
  double stamp_sec{0.0};

  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd effort;

  JointStateMsgData() = default;

  /**
   * @brief Constructs a container sized for `n` joints and zeros numeric fields.
   * @param[in] n Joint count.
   * @throws std::invalid_argument If `n < 0`.
   */
  explicit JointStateMsgData(Eigen::Index n) { resize(n); }

  /**
   * @brief Resizes the container for `n` joints and zeros numeric fields.
   * @param[in] n Joint count.
   * @throws std::invalid_argument If `n < 0`.
   */
  void resize(Eigen::Index n) {
    if (n < 0) throw std::invalid_argument("JointStateMsgData::resize: n < 0");
    name.resize(static_cast<std::size_t>(n));

    position.setZero(n);
    velocity.setZero(n);
    effort.setZero(n);
  }

  /**
   * @brief Returns the number of joints in this container.
   * @return Joint count, equal to `position.size()`.
   */
  Eigen::Index size() const noexcept { return position.size(); }
};

/**
 * @brief Preallocated joint effort command container matching `rlc_interfaces::msg::JointEffortCmd`.
 *
 * @details
 * Sizes:
 * - `name`: length = `size()` (may be left unset by converters).
 * - `effort`: Joint effort/torque command [N·m], size = `size()`.
 *
 * Timestamp:
 * - `stamp_sec`: Message stamp [s], typically derived from `header.stamp`.
 */
struct JointEffortCmdMsgData {
  std::vector<std::string> name;
  double stamp_sec{0.0};
  Eigen::VectorXd effort;

  JointEffortCmdMsgData() = default;

  /**
   * @brief Constructs a container sized for `n` joints and zeros numeric fields.
   * @param[in] n Joint count.
   * @throws std::invalid_argument If `n < 0`.
   */
  explicit JointEffortCmdMsgData(Eigen::Index n) { resize(n); }

  /**
   * @brief Resizes the container for `n` joints and zeros numeric fields.
   * @param[in] n Joint count.
   * @throws std::invalid_argument If `n < 0`.
   */
  void resize(Eigen::Index n) { 
    if (n < 0)
      throw std::invalid_argument("JointEffortCmdMsgData::resize: n < 0");
    name.resize(static_cast<std::size_t>(n));
    effort.setZero(n); 
  }

  /**
   * @brief Returns the number of joints in this container.
   * @return Joint count, equal to `effort.size()`.
   */
  Eigen::Index size() const noexcept { return effort.size(); }
};

/**
 * @brief Preallocated controller state container matching `control_msgs::msg::JointTrajectoryControllerState`.
 *
 * @details
 * Sizes:
 * - `name`: length = `size()` (may be left unset by converters).
 * - `position`, `velocity`, `acceleration`: Feedback [rad], [rad/s], [rad/s^2], size = `size()`.
 * - `position_des`, `velocity_des`, `acceleration_des`: Reference [rad], [rad/s], [rad/s^2], size = `size()`.
 * - `effort`: Output effort/torque [N·m], size = `size()`.
 * - `position_error`, `velocity_error`: Error terms, size = `size()`.
 *
 * Timestamp:
 * - `stamp_sec`: Message stamp [s], typically derived from `header.stamp`.
 */
struct JointControllerStateMsgData {
  std::vector<std::string> name;
  double stamp_sec{0.0};

  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;

  Eigen::VectorXd position_des;
  Eigen::VectorXd velocity_des;
  Eigen::VectorXd acceleration_des;

  Eigen::VectorXd effort;

  Eigen::VectorXd position_error;
  Eigen::VectorXd velocity_error;

  JointControllerStateMsgData() = default;

  /**
   * @brief Constructs a container sized for `n` joints and zeros numeric fields.
   * @param[in] n Joint count.
   * @throws std::invalid_argument If `n < 0`.
   */
  explicit JointControllerStateMsgData(Eigen::Index n) { resize(n); }

  /**
   * @brief Resizes the container for `n` joints and zeros numeric fields.
   * @param[in] n Joint count.
   * @throws std::invalid_argument If `n < 0`.
   */
  void resize(Eigen::Index n) {
    if (n < 0)
      throw std::invalid_argument("JointControllerStateMsgData::resize: n < 0");
    name.resize(static_cast<std::size_t>(n));

    position.setZero(n);
    velocity.setZero(n);
    acceleration.setZero(n);
    position_des.setZero(n);
    velocity_des.setZero(n);
    acceleration_des.setZero(n);
    effort.setZero(n);
    position_error.setZero(n);
    velocity_error.setZero(n);
  }

  /**
   * @brief Returns the number of joints in this container.
   * @return Joint count, equal to `position.size()`.
   */
  Eigen::Index size() const noexcept { return position.size(); }

  /**
   * @brief Recomputes `position_error` and `velocity_error` from current and desired values.
   * @details Computes:
   * - `position_error = position_des - position`
   * - `velocity_error = velocity_des - velocity`
   */
  void recompute_errors() {
    position_error.noalias() = position_des - position;
    velocity_error.noalias() = velocity_des - velocity;
  }
};

}  // namespace rlc_utils::types
