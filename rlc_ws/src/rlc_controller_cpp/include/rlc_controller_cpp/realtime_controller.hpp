#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rbt_core_cpp/robot.hpp"
#include "rlc_utils/types.hpp"

namespace rlc_controller_cpp {

/**
 * @brief ros2_control effort controller plugin using `rbt_core_cpp::Robot`.
 *
 * @details
 * Reads per-joint state interfaces: position, velocity
 * Writes per-joint command interfaces: effort
 * Allocates and validates everything during lifecycle transitions
 * `update()` is allocation-free and performs only: read state, run model, write
 * effort
 */
class RealtimeController final
    : public controller_interface::ControllerInterface {
 public:
  RealtimeController();
  ~RealtimeController() override = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  struct JointStateInterfaces {
    hardware_interface::LoanedStateInterface* position{nullptr};
    hardware_interface::LoanedStateInterface* velocity{nullptr};
  };

  struct JointCommandInterfaces {
    hardware_interface::LoanedCommandInterface* effort{nullptr};
  };

 private:
  controller_interface::CallbackReturn init_interface_handles();

  controller_interface::CallbackReturn calibrate();
  controller_interface::return_type read_state(const rclcpp::Time& time);
  controller_interface::return_type write_cmd();

  std::size_t ndof() const { return ndof_; }
  void set_ndof(std::size_t ndof) { ndof_ = ndof; }

 private:
  std::size_t ndof_{0};
  std::vector<std::string> joints_;
  std::vector<JointStateInterfaces> state_handles_;
  std::vector<JointCommandInterfaces> command_handles_;

  std::optional<rbt_core_cpp::Robot> robot_;

  rlc_utils::types::JointStateMsgData joint_state_;
  rlc_utils::types::JointEffortCmdMsgData joint_cmd_;

  std::vector<std::string> claimed_state_interfaces_;
  std::vector<std::string> claimed_command_interfaces_;
};

}  // namespace rlc_controller_cpp
