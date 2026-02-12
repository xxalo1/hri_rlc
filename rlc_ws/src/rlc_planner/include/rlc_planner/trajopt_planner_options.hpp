#pragma once

#include <string>

#include <rlc_scene_bridge/scene_bridge_options.hpp>

namespace rlc_planner
{

struct TrajOptPlannerOptions
{
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

