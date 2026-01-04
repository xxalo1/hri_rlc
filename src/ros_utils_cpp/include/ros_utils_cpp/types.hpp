#pragma once
#include <Eigen/Core>

#include <string>
#include <vector>

namespace ros_utils_cpp {

struct JointStateMsgData {
  // If you need joint names, copy/set them once outside the hot path.
  std::vector<std::string> name;

  double stamp_sec{0.0};

  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd effort;

  JointStateMsgData() = default;
  explicit JointStateMsgData(int n) { resize(n); }

  void resize(int n) {
    position.setZero(n);
    velocity.setZero(n);
    effort.setZero(n);
  }

  int size() const noexcept { return static_cast<int>(name.size()); }
};

struct JointEffortCmdMsgData {
  std::vector<std::string> joint_names;
  double stamp_sec{0.0};
  Eigen::VectorXd effort;

  JointEffortCmdMsgData() = default;
  explicit JointEffortCmdMsgData(int n) { resize(n); }

  void resize(int n) { effort.setZero(n); }

  int size() const noexcept { return static_cast<int>(effort.size()); }
};

struct JointControllerStateMsgData {
  std::vector<std::string> joint_names;
  double stamp_sec{0.0};

  // Feedback state
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;

  // Reference/desired state
  Eigen::VectorXd position_des;
  Eigen::VectorXd velocity_des;
  Eigen::VectorXd acceleration_des;

  // Output effort (torque)
  Eigen::VectorXd effort;

  // Convenience caches (can be recomputed)
  Eigen::VectorXd position_error;
  Eigen::VectorXd velocity_error;

  JointControllerStateMsgData() = default;
  explicit JointControllerStateMsgData(int n) { resize(n); }

  void resize(int n) {
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

  int size() const noexcept { return static_cast<int>(position.size()); }

  void recompute_errors() {
    position_error.noalias() = position_des - position;
    velocity_error.noalias() = velocity_des - velocity;
  }
};

}  // namespace ros_utils_cpp
