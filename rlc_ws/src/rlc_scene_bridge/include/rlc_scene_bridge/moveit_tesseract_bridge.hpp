#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace tesseract_environment
{
class Environment;
}

namespace rlc_scene_bridge
{
class TesseractEnvSync;

class MoveItTesseractBridge
{
public:
  using PlanningSceneMsg = moveit_msgs::msg::PlanningScene;
  using PlanningSceneMsgConstPtr = PlanningSceneMsg::ConstSharedPtr;
  using PlanningSceneComponentsMsg = moveit_msgs::msg::PlanningSceneComponents;
  using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;

  struct Options
  {
    std::string scene_topic{ "/move_group/monitored_planning_scene" };
    std::string get_scene_srv{ "/get_planning_scene" };

    uint32_t scene_components{ PlanningSceneComponentsMsg::ROBOT_STATE |
                               PlanningSceneComponentsMsg::ROBOT_STATE_ATTACHED_OBJECTS |
                               PlanningSceneComponentsMsg::WORLD_OBJECT_GEOMETRY };

    std::chrono::milliseconds srv_wait{ std::chrono::milliseconds{ 500 } };

    std::size_t scene_qos_depth{ 25 };
  };

  MoveItTesseractBridge(rclcpp::Node& node,
                        std::shared_ptr<tesseract_environment::Environment> env,
                        Options opt = {});

  ~MoveItTesseractBridge();

  MoveItTesseractBridge(const MoveItTesseractBridge&) = delete;
  MoveItTesseractBridge& operator=(const MoveItTesseractBridge&) = delete;

  bool isSynchronized() const noexcept;

  rclcpp::Time lastUpdateStamp() const noexcept;

  std::shared_ptr<tesseract_environment::Environment> envSnapshot() const;

  void requestSync();

  const Options& options() const noexcept
  { return opt_; }

  rclcpp::CallbackGroup::SharedPtr callbackGroup() const
  { return cbg_; }

private:
  enum class EnvState : uint8_t
  {
    UNINITIALIZED,
    SYNCHRONIZING,
    SYNCHRONIZED
  };

  void onSceneUpdate(PlanningSceneMsgConstPtr msg);

  void onSyncResponse(rclcpp::Client<GetPlanningScene>::SharedFuture future);

  rclcpp::Node& node_;
  rclcpp::Logger logger_;
  Options opt_;

  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::Subscription<PlanningSceneMsg>::SharedPtr scene_sub_;
  rclcpp::Client<GetPlanningScene>::SharedPtr get_scene_client_;

  std::atomic<EnvState> env_state_{ EnvState::UNINITIALIZED };
  std::atomic<int64_t> last_update_ns_{ 0 };

  std::unique_ptr<TesseractEnvSync> env_sync_;

  mutable std::mutex sync_mtx_;
};

}  // namespace rlc_scene_bridge
