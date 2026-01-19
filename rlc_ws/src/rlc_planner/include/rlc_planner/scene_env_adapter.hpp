#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <tesseract_environment/environment.hpp>

namespace planning_scene {
class PlanningScene;
}  // namespace planning_scene

namespace rlc_planner {

/**
 * @brief Build a per-request Tesseract Environment snapshot from a MoveIt
 * PlanningScene.
 *
 * Design (Option 1):
 *   - Maintain an initialized "robot-only" base environment.
 *   - For each request:
 *       1) clone base env
 *       2) add all MoveIt world objects into the clone
 *       3) sync robot joint state into the clone
 *       4) return clone
 *
 * This avoids maintaining a long-lived mirrored environment and avoids
 * diff-sync complexity.
 */
class SceneEnvAdapter {
 public:
  struct Options {
    /**
     * Parent link in the Tesseract environment to which MoveIt "world objects"
     * are attached.
     *
     * If empty, we default to env_robot_base_->getRootLinkName() after
     * initialization.
     */
    std::string world_parent_link;

    /** Prefixes for generated link/joint names for world objects. */
    std::string world_object_link_prefix = "world_obj__";
    std::string world_object_joint_prefix = "joint__world_obj__";

    /** Whether to sanitize MoveIt object ids into safe link/joint name tokens.
     */
    bool sanitize_object_ids = true;
  };

  SceneEnvAdapter() = default;

  /**
   * @brief Initialize the robot-only base environment from URDF/SRDF XML
   * strings.
   * @throws std::runtime_error on failure.
   */
  void initialize(const std::string& urdf_xml, const std::string& srdf_xml,
                  Options options = {});

  /**
   * @brief Create a request-scoped environment snapshot for planning.
   *
   * This function:
   *   - clones the base env
   *   - rebuilds MoveIt world objects into the clone
   *   - sets the robot joint state in the clone
   *
   * @throws std::runtime_error on failure or invalid input.
   */
  tesseract_environment::Environment::UPtr make_snapshot(
      const std::shared_ptr<const planning_scene::PlanningScene>& scene) const;

 private:
  void add_world_objects(tesseract_environment::Environment& env,
                         const planning_scene::PlanningScene& scene) const;

  void sync_robot_state(tesseract_environment::Environment& env,
                        const planning_scene::PlanningScene& scene) const;

  std::shared_ptr<tesseract_environment::Environment> env_robot_base_;
  Options options_;

  mutable std::mutex mutex_;
};

}  // namespace rlc_planner
