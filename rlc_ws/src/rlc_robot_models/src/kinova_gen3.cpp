#include "rlc_robot_models/kinova_gen3.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdexcept>

namespace rlc_robot_models {

static std::string variant_Name(Gen3Variant variant) {
  switch (variant) {
    case Gen3Variant::DOF7_BASE:
      return "DOF7_BASE";
    case Gen3Variant::DOF7_VISION:
      return "DOF7_VISION";
    case Gen3Variant::DOF6_BASE:
      return "DOF6_BASE";
    case Gen3Variant::DOF6_VISION:
      return "DOF6_VISION";
    case Gen3Variant::DOF6_WITH_GRIPPER:
      return "DOF6_WITH_GRIPPER";
    case Gen3Variant::DOF7_WITH_GRIPPER:
      return "DOF7_WITH_GRIPPER";
  }
  throw std::runtime_error("Unknown Gen3Variant");
}

static std::string variant_urdf_stem(Gen3Variant variant) {
  switch (variant) {
    case Gen3Variant::DOF7_BASE:
      return "gen3";
    case Gen3Variant::DOF7_VISION:
      return "gen3_vision";
    case Gen3Variant::DOF6_BASE:
      return "gen3_dof6";
    case Gen3Variant::DOF6_VISION:
      return "gen3_dof6_vision";
    case Gen3Variant::DOF6_WITH_GRIPPER:
      return "gen3_dof6_robotiq_2f_85";
    case Gen3Variant::DOF7_WITH_GRIPPER:
      return "gen3_robotiq_2f_85";
  }
  throw std::runtime_error("Unknown Gen3Variant");
}

std::string gen3_urdf_path(Gen3Variant variant) {
  const auto desc_share =
      ament_index_cpp::get_package_share_directory("rlc_robot_descriptions");
  const auto stem = variant_urdf_stem(variant);
  return desc_share + "/robots/kinova_gen3/urdf/" + stem + ".urdf";
}

rbt_core_cpp::RobotSpec make_gen3_spec(Gen3Variant variant) {
  rbt_core_cpp::RobotSpec spec;
  spec.name = variant_Name(variant);
  spec.urdf = gen3_urdf_path(variant);
  spec.tcp_frame = "tool_frame";
  return spec;
}

rbt_core_cpp::Robot make_gen3_robot(Gen3Variant variant) {
  return rbt_core_cpp::Robot::FromSpec(make_gen3_spec(variant));
}

}  // namespace rlc_robot_models
