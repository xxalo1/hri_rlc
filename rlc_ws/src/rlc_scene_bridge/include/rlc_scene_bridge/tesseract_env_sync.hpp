// rlc_scene_bridge/include/rlc_scene_bridge/tesseract_env_sync.hpp
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <tesseract_environment/fwd.h>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/logger.hpp>

namespace tesseract_environment
{
class Environment;
class EnvironmentMonitorInterface;
class Command;
}  // namespace tesseract_environment

namespace rlc_scene_bridge
{

struct Options
{
  tesseract_scene::NamingPolicy naming_policy;
  std::string world_frame;
  bool allow_replace{ true };
};

class TesseractEnvSync
{
public:
  using PlanningSceneMsg = moveit_msgs::msg::PlanningScene;
  using MonitorInterfaceConstPtr =
      std::shared_ptr<const tesseract_environment::EnvironmentMonitorInterface>;
  using EnvironmentUPtr = std::unique_ptr<tesseract_environment::Environment>;

  using CommandPtr = std::shared_ptr<const tesseract_environment::Command>;
  using CommandList = std::vector<CommandPtr>;

  TesseractEnvSync(MonitorInterfaceConstPtr monitor_interface,
                   std::string monitor_namespace, rclcpp::Logger logger,
                   Options opt = {});

  ~TesseractEnvSync() = default;

  TesseractEnvSync(const TesseractEnvSync&) = delete;
  TesseractEnvSync& operator=(const TesseractEnvSync&) = delete;

  bool isInitialized() const noexcept;

  bool applyFullScene(const PlanningSceneMsg& scene);

  bool applyDiff(const PlanningSceneMsg& scene);

  EnvironmentUPtr snapshot() const;

  const std::string& monitorNamespace() const noexcept
  {
    return monitor_namespace_;
  };

  const Options& options() const noexcept
  {
    return opt_;
  }

private:
  bool fromFullScene(const PlanningSceneMsg& scene);

  bool fromDiff(const PlanningSceneMsg& scene);

  bool applyCommands(const tesseract_environment::Commands& commands) const;

  CommandPtr makeAddWorldCommand(const moveit_msgs::msg::CollisionObject& object);

  CommandPtr makeRemoveWorldCommand(const moveit_msgs::msg::CollisionObject& object);

  CommandPtr makeMoveWorldCommand(const moveit_msgs::msg::CollisionObject& object);

  CommandPtr makeAddAttachedCommand(const moveit_msgs::msg::AttachedCollisionObject& aco);

  CommandPtr makeRemoveAttachedCommand(const moveit_msgs::msg::CollisionObject& object);
  
  bool appendRemoveAttachedCommands(const moveit_msgs::msg::AttachedCollisionObject& aco,
                                    tesseract_environment::Commands& commands);

  MonitorInterfaceConstPtr monitor_interface_;
  std::string monitor_namespace_;
  rclcpp::Logger logger_;
  Options opt_;

  std::unordered_map<std::string, tesseract_scene::ObjectInfo> world_objects_;
  std::unordered_map<std::string, tesseract_scene::ObjectInfo> attached_objects_;
};

}  // namespace rlc_scene_bridge
