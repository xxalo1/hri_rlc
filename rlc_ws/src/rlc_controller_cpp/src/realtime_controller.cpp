#include "rlc_controller_cpp/realtime_controller.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rlc_robot_models/kinova_gen3.hpp"

namespace rlc_controller_cpp {

RealtimeController::RealtimeController() = default;

controller_interface::CallbackReturn RealtimeController::on_init() {
  auto_declare<std::vector<std::string>>("joints", {});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RealtimeController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(joints_.size());

  for (const auto &j : joints_) {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
RealtimeController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(joints_.size() * 2);

  for (const auto &joint : joints_) {
    cfg.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return cfg;
}

controller_interface::CallbackReturn RealtimeController::on_configure(
    const rclcpp_lifecycle::State &) {
  ifaces_.clear();

  joints_ = get_node()->get_parameter("joints").as_string_array();
  if (joints_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Parameter 'joints' must be set and non-empty");
    return controller_interface::CallbackReturn::FAILURE;
  }

  try {
    robot_.emplace(rlc_robot_models::make_gen3_spec());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to construct robot model: %s", e.what());
    return controller_interface::CallbackReturn::FAILURE;
  }

  const auto n_model = static_cast<std::size_t>(robot_->n());
  if (n_model != joints_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Robot DOF mismatch: robot.n()=%zu, joints.size()=%zu",
                 n_model, joints_.size());
    return controller_interface::CallbackReturn::FAILURE;
  }

  const Eigen::Index n = static_cast<Eigen::Index>(joints_.size());

  joint_state_.resize(n);
  joint_cmd_.resize(n);

  joint_state_.name = joints_;
  joint_cmd_.name = joints_;

  try {
    robot_->configure_io(joints_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_.configure_io failed: %s",
                 e.what());
    return controller_interface::CallbackReturn::FAILURE;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RealtimeController::validate_and_cache_interfaces_() {
  if (!robot_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Controller activated before robot model was configured");
    return controller_interface::CallbackReturn::FAILURE;
  }

  const std::size_t n = joints_.size();
  if (state_interfaces_.size() != n * 2) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu state interfaces (pos+vel), got %zu", n * 2,
                 state_interfaces_.size());
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (command_interfaces_.size() != n) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu command interfaces (effort), got %zu", n,
                 command_interfaces_.size());
    return controller_interface::CallbackReturn::FAILURE;
  }

  for (std::size_t i = 0; i < n; ++i) {
    const auto expected_pos =
        joints_[i] + "/" + hardware_interface::HW_IF_POSITION;
    const auto expected_vel =
        joints_[i] + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto expected_eff =
        joints_[i] + "/" + hardware_interface::HW_IF_EFFORT;

    if (state_interfaces_[2 * i].get_name() != expected_pos) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "State interface[%zu] mismatch: expected '%s', got '%s'",
                   2 * i, expected_pos.c_str(),
                   state_interfaces_[2 * i].get_name().c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    if (state_interfaces_[2 * i + 1].get_name() != expected_vel) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "State interface[%zu] mismatch: expected '%s', got '%s'",
                   2 * i + 1, expected_vel.c_str(),
                   state_interfaces_[2 * i + 1].get_name().c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    if (command_interfaces_[i].get_name() != expected_eff) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Command interface[%zu] mismatch: expected '%s', got '%s'",
                   i, expected_eff.c_str(),
                   command_interfaces_[i].get_name().c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  ifaces_.pos.resize(n);
  ifaces_.vel.resize(n);
  ifaces_.eff.resize(n);

  for (std::size_t i = 0; i < n; ++i) {
    ifaces_.pos[i] = 2 * i;
    ifaces_.vel[i] = 2 * i + 1;
    ifaces_.eff[i] = i;
  }

  ifaces_.ready = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

void RealtimeController::write_zero_effort_() {
  for (auto &cmd : command_interfaces_) {
    (void)cmd.set_value(0.0, 1);
  }
}

controller_interface::CallbackReturn RealtimeController::on_activate(
    const rclcpp_lifecycle::State &) {
  const auto rc = validate_and_cache_interfaces_();
  if (rc != controller_interface::CallbackReturn::SUCCESS) {
    ifaces_.clear();
    return rc;
  }

  write_zero_effort_();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RealtimeController::on_deactivate(
    const rclcpp_lifecycle::State &) {
  write_zero_effort_();
  ifaces_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RealtimeController::update(
    const rclcpp::Time &time, const rclcpp::Duration &) {
  if (!ifaces_.ready || !robot_) {
    return controller_interface::return_type::ERROR;
  }

  const std::size_t n = joints_.size();

  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Index ii = static_cast<Eigen::Index>(i);

    const auto pos = state_interfaces_[ifaces_.pos[i]].get_optional<double>(1);
    const auto vel = state_interfaces_[ifaces_.vel[i]].get_optional<double>(1);
    if (!pos || !vel) {
      return controller_interface::return_type::ERROR;
    }

    joint_state_.position[ii] = *pos;
    joint_state_.velocity[ii] = *vel;
  }

  joint_state_.stamp_sec = time.seconds();

  robot_->set_joint_state(&joint_state_.position, &joint_state_.velocity,
                          nullptr, &joint_state_.stamp_sec);

  joint_cmd_.effort.noalias() = robot_->compute_ctrl_effort();
  joint_cmd_.stamp_sec = joint_state_.stamp_sec;

  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Index ii = static_cast<Eigen::Index>(i);
    if (!command_interfaces_[ifaces_.eff[i]].set_value(joint_cmd_.effort[ii],
                                                       1)) {
      return controller_interface::return_type::ERROR;
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace rlc_controller_cpp

PLUGINLIB_EXPORT_CLASS(rlc_controller_cpp::RealtimeController,
                       controller_interface::ControllerInterface)
