#pragma once
#include <Eigen/Core>
#include <string>
#include <vector>

namespace rlc_utils::types {

struct JointStateMsgData {
  std::vector<std::string> name;
  double stamp_sec{0.0};

  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd effort;

  JointStateMsgData() = default;
  explicit JointStateMsgData(Eigen::Index n) { resize(n); }

  void resize(Eigen::Index n) {
    if (n < 0) throw std::invalid_argument("JointStateMsgData::resize: n < 0");
    name.resize(static_cast<std::size_t>(n));

    position.setZero();
    velocity.setZero();
    effort.setZero();
  }

  Eigen::Index size() const noexcept { return position.size(); }
};

struct JointEffortCmdMsgData {
  std::vector<std::string> name;
  double stamp_sec{0.0};
  Eigen::VectorXd effort;

  JointEffortCmdMsgData() = default;
  explicit JointEffortCmdMsgData(Eigen::Index n) { resize(n); }

  void resize(Eigen::Index n) { 
    if (n < 0)
      throw std::invalid_argument("JointEffortCmdMsgData::resize: n < 0");
    name.resize(static_cast<std::size_t>(n));
    effort.setZero(n); 
  }

  Eigen::Index size() const noexcept { return effort.size(); }
};

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
  explicit JointControllerStateMsgData(Eigen::Index n) { resize(n); }
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

  Eigen::Index size() const noexcept { return position.size(); }

  void recompute_errors() {
    position_error.noalias() = position_des - position;
    velocity_error.noalias() = velocity_des - velocity;
  }
};

}  // namespace rlc_utils
