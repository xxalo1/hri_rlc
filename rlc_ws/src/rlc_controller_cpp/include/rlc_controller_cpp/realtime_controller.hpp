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

 private:
  struct IfaceIndexCache {
    std::vector<std::size_t> pos;
    std::vector<std::size_t> vel;
    std::vector<std::size_t> eff;
    bool ready{false};

    void clear() {
      pos.clear();
      vel.clear();
      eff.clear();
      ready = false;
    }
  };

 private:
  controller_interface::CallbackReturn validate_and_cache_interfaces_();

  void write_zero_effort_();

 private:
  std::vector<std::string> joints_;

  std::optional<rbt_core_cpp::Robot> robot_;

  rlc_utils::types::JointStateMsgData joint_state_;
  rlc_utils::types::JointEffortCmdMsgData joint_cmd_;

  IfaceIndexCache ifaces_{};
};

}  // namespace rlc_controller_cpp
