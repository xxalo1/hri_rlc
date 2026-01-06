#include "rlc_controller_cpp/realtime_controller.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <exception>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rlc_robot_models/kinova_gen3.hpp"

namespace rlc_controller_cpp {

namespace ci = controller_interface;
using Self = RealtimeController;
using CallbackReturn = ci::CallbackReturn;

RealtimeController::RealtimeController() = default;

ci::CallbackReturn Self::on_init() {
  auto_declare<std::vector<std::string>>("joints", {});
  return ci::CallbackReturn::SUCCESS;
}

ci::InterfaceConfiguration Self::command_interface_configuration() const {
  ci::InterfaceConfiguration cfg;
  cfg.type = ci::interface_configuration_type::INDIVIDUAL;
  cfg.names = claimed_command_interfaces_;
  return cfg;
}

ci::InterfaceConfiguration Self::state_interface_configuration() const {
  ci::InterfaceConfiguration cfg;
  cfg.type = ci::interface_configuration_type::INDIVIDUAL;
  cfg.names = claimed_state_interfaces_;
  return cfg;
}

ci::CallbackReturn Self::on_configure(const rclcpp_lifecycle::State&) {
  joints_ = get_node()->get_parameter("joints").as_string_array();
  if (joints_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Parameter 'joints' must be set and non-empty");
    return ci::CallbackReturn::FAILURE;
  }

  set_ndof(joints_.size());
  const std::size_t n = ndof();

  claimed_state_interfaces_.reserve(n * 2);
  claimed_command_interfaces_.reserve(n);

  for (const auto& j : joints_) {
    claimed_state_interfaces_.emplace_back(j + "/" +
                                           hardware_interface::HW_IF_POSITION);
    claimed_state_interfaces_.emplace_back(j + "/" +
                                           hardware_interface::HW_IF_VELOCITY);
    claimed_command_interfaces_.emplace_back(j + "/" +
                                             hardware_interface::HW_IF_EFFORT);
  }

  try {
    robot_.emplace(rlc_robot_models::make_gen3_spec());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to construct robot model: %s", e.what());
    return ci::CallbackReturn::FAILURE;
  }

  const auto n_model = static_cast<std::size_t>(robot_->n());
  if (n_model != n) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Robot DOF mismatch: robot.n()=%zu, joints.size()=%zu",
                 n_model, n);
    return ci::CallbackReturn::FAILURE;
  }

  const Eigen::Index ni = static_cast<Eigen::Index>(n);

  joint_state_.resize(ni);
  joint_cmd_.resize(ni);

  joint_state_.name = joints_;
  joint_cmd_.name = joints_;

  try {
    robot_->configure_io(joints_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_.configure_io failed: %s",
                 e.what());
    return ci::CallbackReturn::FAILURE;
  }

  return ci::CallbackReturn::SUCCESS;
}

ci::CallbackReturn Self::init_interface_handles() {
  const std::size_t n = ndof();

  state_handles_.assign(n, JointStateInterfaces{});
  command_handles_.assign(n, JointCommandInterfaces{});

  auto find_state = [this](const std::string& name)
      -> hardware_interface::LoanedStateInterface* {
    for (auto& si : state_interfaces_) {
      if (si.get_name() == name) return &si;
    }
    return nullptr;
  };

  auto find_command = [this](const std::string& name)
      -> hardware_interface::LoanedCommandInterface* {
    for (auto& ci : command_interfaces_) {
      if (ci.get_name() == name) return &ci;
    }
    return nullptr;
  };

  for (std::size_t i = 0; i < n; ++i) {
    const auto& position_name = claimed_state_interfaces_[2 * i];
    const auto& velocity_name = claimed_state_interfaces_[2 * i + 1];
    const auto& command_name = claimed_command_interfaces_[i];

    state_handles_[i].position = find_state(position_name);
    if (!state_handles_[i].position) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface: %s",
                   position_name.c_str());
      return ci::CallbackReturn::FAILURE;
    }

    state_handles_[i].velocity = find_state(velocity_name);
    if (!state_handles_[i].velocity) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface: %s",
                   velocity_name.c_str());
      return ci::CallbackReturn::FAILURE;
    }

    command_handles_[i].effort = find_command(command_name);
    if (!command_handles_[i].effort) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing command interface: %s",
                   command_name.c_str());
      return ci::CallbackReturn::FAILURE;
    }
  }

  return ci::CallbackReturn::SUCCESS;
}

ci::CallbackReturn Self::calibrate() {
  const rclcpp::Time time = get_node()->now();
  const auto rc = read_state(time);
  if (rc != ci::return_type::OK) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to read state during calibration");
    return ci::CallbackReturn::FAILURE;
  }
  robot_->set_joint_des(&joint_state_.position);
  robot_->set_joint_state(&joint_state_.position, &joint_state_.velocity,
                          nullptr, &joint_state_.stamp_sec);
  joint_cmd_.effort = robot_->compute_ctrl_effort();

  joint_cmd_.stamp_sec = joint_state_.stamp_sec;

  const auto rc_write = write_cmd();
  if (rc_write != ci::return_type::OK) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to write command during calibration");
    return ci::CallbackReturn::FAILURE;
  }

  return ci::CallbackReturn::SUCCESS;
}

ci::CallbackReturn Self::on_activate(const rclcpp_lifecycle::State&) {
  auto rc = init_interface_handles();
  if (rc != ci::CallbackReturn::SUCCESS) {
    return rc;
  }
  rc = calibrate();
  return rc;
}

ci::CallbackReturn Self::on_deactivate(const rclcpp_lifecycle::State&) {
  state_handles_.clear();
  command_handles_.clear();
  return ci::CallbackReturn::SUCCESS;
}

ci::return_type Self::read_state(const rclcpp::Time& time) {
  const std::size_t n = ndof();
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Index ii = static_cast<Eigen::Index>(i);
    const auto pos = state_handles_[i].position->get_optional<double>(1);
    const auto vel = state_handles_[i].velocity->get_optional<double>(1);

    if (!pos || !vel) return ci::return_type::ERROR;

    joint_state_.position[ii] = *pos;
    joint_state_.velocity[ii] = *vel;
  }
  joint_state_.stamp_sec = time.seconds();
  return ci::return_type::OK;
}

ci::return_type Self::write_cmd() {
  const std::size_t n = ndof();
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Index ii = static_cast<Eigen::Index>(i);
    const bool success =
        command_handles_[i].effort->set_value(joint_cmd_.effort[ii], 1);
    if (!success) return ci::return_type::ERROR;
  }
  return ci::return_type::OK;
}

ci::return_type Self::update(const rclcpp::Time& time,
                             const rclcpp::Duration&) {
  read_state(time);

  robot_->set_joint_state(&joint_state_.position, &joint_state_.velocity,
                          nullptr, &joint_state_.stamp_sec);
  joint_cmd_.effort.noalias() = robot_->compute_ctrl_effort();
  joint_cmd_.stamp_sec = joint_state_.stamp_sec;

  write_cmd();

  return ci::return_type::OK;
}

}  // namespace rlc_controller_cpp

PLUGINLIB_EXPORT_CLASS(rlc_controller_cpp::RealtimeController,
                       controller_interface::ControllerInterface)
