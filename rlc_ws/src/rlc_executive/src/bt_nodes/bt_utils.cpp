#include "rlc_executive/bt_nodes/bt_utils.hpp"

#include <cstddef>
#include <exception>
#include <utility>

#include <behaviortree_cpp/exceptions.h>
#include <rclcpp/duration.hpp>

#include "rlc_executive/core/blackboard_keys.hpp"
#include "rlc_executive/core/runtime_context.hpp"

namespace rlc_executive
{
namespace bt_utils
{

void ensureContextAndLogger(const BT::TreeNode& node,
                            std::shared_ptr<RuntimeContext>& ctx_cache,
                            std::optional<rclcpp::Logger>& logger_cache)
{
  if (!ctx_cache)
  {
    try
    {
      if (!node.config().blackboard)
      {
        throw BT::RuntimeError(node.registrationName(), "[", node.fullPath(),
                               "]: blackboard is null");
      }

      ctx_cache = node.config().blackboard->get<std::shared_ptr<RuntimeContext>>(
          bb::RUNTIME_CONTEXT);
    }
    catch (const std::exception& e)
    {
      throw BT::RuntimeError(node.registrationName(), "[", node.fullPath(),
                             "]: failed to read RuntimeContext: ", e.what());
    }
  }

  if (!ctx_cache)
  {
    throw BT::RuntimeError(node.registrationName(), "[", node.fullPath(),
                           "]: RuntimeContext missing on blackboard");
  }

  if (!logger_cache)
  {
    rclcpp::Logger base_logger = ctx_cache->node().get_logger();

    base_logger = base_logger.get_child(node.registrationName());
    base_logger = base_logger.get_child(sanitizeLoggerName(node.fullPath()));

    logger_cache = std::move(base_logger);
  }
}

std::string sanitizeLoggerName(std::string s)
{
  for (char& c : s)
  {
    if (c == '/')
    {
      c = '.';
      continue;
    }

    const bool ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
                    (c >= 'a' && c <= 'z') || (c == '.') || (c == '_');
    if (!ok)
    {
      c = '_';
    }
  }
  return s;
}

void setDiagnostics(BT::TreeNode& node, const std::string& error,
                    const std::string& feedback, double elapsed_sec)
{
  setOutput(node, DiagnosticPortKeys::ERROR, error);
  setOutput(node, DiagnosticPortKeys::FEEDBACK, feedback);
  setOutput(node, DiagnosticPortKeys::ELAPSED_SEC, elapsed_sec);
}

void setRunningDiagnostics(BT::TreeNode& node, const std::string& feedback,
                           double elapsed_sec)
{
  setDiagnostics(node, std::string(), feedback, elapsed_sec);
}

void setFailureDiagnostics(BT::TreeNode& node, const std::string& error,
                           double elapsed_sec)
{
  setDiagnostics(node, error, std::string(), elapsed_sec);
}

void setSuccessDiagnostics(BT::TreeNode& node, double elapsed_sec)
{
  setDiagnostics(node, std::string(), std::string(), elapsed_sec);
}

rbt_types::Trajectory
fromRobotTrajectory(const moveit_msgs::msg::RobotTrajectory& robot_trajectory)
{
  const auto& joint_trajectory = robot_trajectory.joint_trajectory;
  const auto& points = joint_trajectory.points;
  const std::size_t num_joints = points.empty() ? 0U : points.front().positions.size();

  rbt_types::Trajectory trajectory;
  trajectory.states.resize(static_cast<Eigen::Index>(points.size()),
                           static_cast<Eigen::Index>(num_joints));

  for (std::size_t row = 0; row < points.size(); ++row)
  {
    for (std::size_t col = 0; col < points[row].positions.size(); ++col)
    {
      trajectory.states(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
          points[row].positions[col];
    }
  }

  trajectory.dt = 1.0;
  if (points.size() > 1)
  {
    const double total_duration = (rclcpp::Duration(points.back().time_from_start) -
                                   rclcpp::Duration(points.front().time_from_start))
                                      .seconds();
    trajectory.dt = total_duration / static_cast<double>(points.size() - 1U);
    if (!(trajectory.dt > 0.0))
    {
      trajectory.dt = 1.0;
    }
  }

  return trajectory;
}

moveit_msgs::msg::RobotTrajectory
toRobotTrajectory(const rbt_types::Trajectory& trajectory,
                  const std::vector<std::string>& joint_names)
{
  moveit_msgs::msg::RobotTrajectory robot_trajectory;
  auto& joint_trajectory = robot_trajectory.joint_trajectory;

  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.resize(trajectory.length());

  for (std::size_t row = 0; row < trajectory.length(); ++row)
  {
    auto& point = joint_trajectory.points[row];
    point.positions.resize(trajectory.dim());

    for (std::size_t col = 0; col < trajectory.dim(); ++col)
    {
      point.positions[col] = trajectory.states(static_cast<Eigen::Index>(row),
                                               static_cast<Eigen::Index>(col));
    }

    point.time_from_start =
        rclcpp::Duration::from_seconds(static_cast<double>(row) * trajectory.dt).to_msg();
  }

  return robot_trajectory;
}

}  // namespace bt_utils
}  // namespace rlc_executive
