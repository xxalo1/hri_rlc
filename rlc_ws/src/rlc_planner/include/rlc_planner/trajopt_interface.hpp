#pragma once

#include <rlc_planner/scene_env_adapter.hpp>

#include <moveit/planning_interface/planning_interface.hpp>
#include <rclcpp/node.hpp>

#include <mutex>
#include <string>

namespace rlc_planner {

class TrajOptInterface {
 public:
  struct Options {
    int default_num_steps = 20;
  };

  TrajOptInterface(const rclcpp::Node::SharedPtr& node,
                   std::string parameter_namespace, Options options = {});

  bool plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
            const planning_interface::MotionPlanRequest& req,
            planning_interface::MotionPlanResponse& res);

 private:
  bool ensure_initialized(std::string& error);
  int resolve_num_steps() const;

  rclcpp::Node::SharedPtr node_;
  std::string parameter_namespace_;
  Options options_;

  SceneEnvAdapter env_adapter_;
  bool env_adapter_initialized_{false};
  std::mutex init_mutex_;
};

}  // namespace rlc_planner
