#pragma once

/**
 * @file trajopt_planner_options.hpp
 * @brief Parameter-backed option structs for the TrajOpt MoveIt planner plugin.
 */

#include <string>

#include <rlc_scene_bridge/scene_bridge_options.hpp>

namespace rlc_planner
{

/**
 * @brief Aggregated options for the `rlc_trajopt` MoveIt planner plugin.
 *
 * @details
 * - `scene_bridge` configures the MoveIt↔Tesseract scene/environment synchronization.
 * - `trajopt` configures TrajOpt-specific solve settings and caching behavior.
 */
struct TrajOptPlannerOptions
{
  /**
   * @brief TrajOpt interface options that are fixed at initialization time.
   *
   * @details
   * - `default_num_steps`: Default number of waypoints in the optimization (must be >= 2).
   * - `use_moveit_group_as_tesseract_manipulator`: If true, the MoveIt `req.group_name`
   *   is used as the Tesseract manipulator group.
   * - `fixed_tesseract_manipulator_group`: Used when
   *   `use_moveit_group_as_tesseract_manipulator` is false (falls back to `"manipulator"`
   *   if empty).
   * - `enable_seed_cache`: Enables caching of seed hints across calls.
   * - `enable_last_solution_cache`: Enables caching of last solutions across calls.
   */
  struct TrajOptOptions
  {
    int default_num_steps{ 20 };

    bool use_moveit_group_as_tesseract_manipulator{ true };
    std::string fixed_tesseract_manipulator_group{ "manipulator" };

    bool enable_seed_cache{ false };
    bool enable_last_solution_cache{ false };
  };

  rlc_scene_bridge::MoveItTesseractBridgeOptions scene_bridge{};
  TrajOptOptions trajopt{};
};

}  // namespace rlc_planner
