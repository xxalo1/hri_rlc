#pragma once

/**
 * @file kinova_gen3.hpp
 * @brief Factory functions and configuration for Kinova Gen3 robot models.
 *
 * Provides utilities to instantiate RobotSpec and Robot objects for various
 * Kinova Gen3 arm configurations (6-DOF and 7-DOF, with or without end-effector
 * accessories).
 */

#include <string>

#include "rbt_core_cpp/robot.hpp"

namespace rlc_robot_models {

/**
 * @enum Gen3Variant
 * @brief Enumerates the supported Kinova Gen3 robot configurations.
 */
enum class Gen3Variant {
  DOF7_BASE,          ///< 7-DOF arm, base configuration (no camera module).
  DOF7_VISION,        ///< 7-DOF arm with vision (depth camera) module.
  DOF6_BASE,          ///< 6-DOF arm, base configuration (no camera module).
  DOF6_VISION,        ///< 6-DOF arm with vision (depth camera) module.
  DOF6_WITH_GRIPPER,  ///< 6-DOF arm with Robotiq gripper attached.
  DOF7_WITH_GRIPPER,  ///< 7-DOF arm with Robotiq gripper attached.
};

/**
 * @brief Returns the filesystem path to the URDF for a given Gen3 variant.
 * @param variant The robot configuration to retrieve (default: DOF7_VISION).
 * @return Absolute path to the corresponding URDF file.
 */
std::string gen3_urdf_path(Gen3Variant variant = Gen3Variant::DOF7_VISION);

/**
 * @brief Constructs a RobotSpec for the specified Gen3 variant.
 * @param variant The robot configuration (default: DOF7_VISION).
 * @return A RobotSpec populated with kinematic and dynamic parameters.
 */
rbt_core_cpp::RobotSpec make_gen3_spec(
    Gen3Variant variant = Gen3Variant::DOF7_VISION);

/**
 * @brief Constructs a fully initialized Robot instance for the specified Gen3 variant.
 * @param variant The robot configuration (default: DOF7_VISION).
 * @return A Robot object ready for kinematics/dynamics computations.
 */
rbt_core_cpp::Robot make_gen3_robot(
    Gen3Variant variant = Gen3Variant::DOF7_VISION);

}  // namespace rlc_robot_models
