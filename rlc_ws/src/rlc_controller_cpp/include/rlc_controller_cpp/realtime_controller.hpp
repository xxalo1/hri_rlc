#pragma once

#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"

#include "rbt_core_cpp/robot.hpp"
#include "rlc_utils/types.hpp"

namespace rlc_controller_cpp
{

/**
 * @brief ros2_control effort controller plugin using `rbt_core_cpp::Robot`.
 *
 * @details
 * This controller reads joint position and velocity from ros2_control state interfaces and writes
 * joint effort commands to ros2_control command interfaces.
 *
 * The realtime update loop (`update()`) is allocation-free: all internal buffers are preallocated
 * during `on_configure()`.
 */
class RealtimeController final : public controller_interface::ControllerInterface
{
public:
  RealtimeController();
  ~RealtimeController() override = default;

  /**
   * @brief Declares controller parameters.
   * @return Lifecycle callback return status.
   */
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Returns command interfaces required by this controller.
   * @return Command interface configuration (individual effort interfaces).
   */
  controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  /**
   * @brief Returns state interfaces required by this controller.
   * @return State interface configuration (individual position/velocity interfaces).
   */
  controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  /**
   * @brief Configures the controller from parameters and preallocates buffers.
   * @param[in] previous_state Previous lifecycle state.
   * @return Lifecycle callback return status.
   */
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Activates the controller and zeros effort outputs.
   * @param[in] previous_state Previous lifecycle state.
   * @return Lifecycle callback return status.
   */
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Deactivates the controller and zeros effort outputs.
   * @param[in] previous_state Previous lifecycle state.
   * @return Lifecycle callback return status.
   */
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Performs one control update step.
   * @param[in] time Time at the start of this control iteration.
   * @param[in] period Time since the last control iteration.
   * @return OK on success; ERROR on failure.
   */
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

private:
  std::vector<std::string> joints_;

  std::optional<rbt_core_cpp::Robot> robot_;

  rlc_utils::types::JointStateMsgData joint_state_;
  rlc_utils::types::JointEffortCmdMsgData joint_cmd_;
};

}  // namespace rlc_controller_cpp
