#pragma once

#include <string>

#include "rbt_core_cpp/robot.hpp"

namespace rlc_robot_models {

enum class Gen3Variant {
  DOF7_BASE,
  DOF7_VISION,
  DOF6_BASE,
  DOF6_VISION,
  DOF6_WITH_GRIPPER,
  DOF7_WITH_GRIPPER,
};

std::string Gen3UrdfPath(Gen3Variant variant = Gen3Variant::DOF7_VISION);

rbt_core_cpp::RobotSpec MakeGen3Spec(Gen3Variant variant = Gen3Variant::DOF7_VISION);

rbt_core_cpp::Robot MakeGen3Robot(Gen3Variant variant = Gen3Variant::DOF7_VISION);

}  // namespace rlc_robot_models
