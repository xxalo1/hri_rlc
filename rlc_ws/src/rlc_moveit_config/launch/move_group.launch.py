#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# NOTE: Modified from upstream franka_ros2 by hri_rlc.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

_THIS_DIR = os.path.dirname(__file__)
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from move_group_utils import (
    load_yaml,
    make_move_group_node,
    make_tesseract_environment_monitor_node,
    planning_plugins_for,
    Planner,
)


def generate_launch_description() -> LaunchDescription:
    """
    Launch the MoveIt `move_group` node (and optional Tesseract monitor).

    Parameters are declared as launch arguments so another launch file can
    include this launch file and override:
    - which MoveIt config YAML files to use
    - which planner pipeline to enable (default: OMPL)

    Notes
    -----
    When `planner:=rlc_trajopt`, this launch also starts a
    `tesseract_monitoring_environment_node` because `rlc_planner` connects to it
    via `tesseract_monitoring::ROSEnvironmentMonitorInterface`.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    namespace = LaunchConfiguration("namespace")
    planner = LaunchConfiguration("planner_plugin")
    tesseract_monitor_namespace = LaunchConfiguration("tesseract_monitor_namespace")
    tesseract_joint_state_topic = LaunchConfiguration("tesseract_joint_state_topic")

    moveit_config_package = LaunchConfiguration("moveit_config_package")
    kinematics_yaml = LaunchConfiguration("kinematics_yaml")
    joint_limits_yaml = LaunchConfiguration("joint_limits_yaml")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    ompl_planning_yaml = LaunchConfiguration("ompl_planning_yaml")
    trajopt_planning_yaml = LaunchConfiguration("trajopt_planning_yaml")

    declared_arguments = [
        DeclareLaunchArgument(
            "db", default_value="false", description="Database flag."
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time if true.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for MoveIt nodes.",
        ),
        DeclareLaunchArgument(
            "planner",
            default_value="ompl",
            description="Planner pipeline selection (ompl | rlc_trajopt).",
        ),
        DeclareLaunchArgument(
            "tesseract_monitor_namespace",
            default_value="tesseract_monitor",
            description=(
                "Tesseract monitor namespace used for services/topics (e.g., "
                '"/<ns>/get_tesseract_information"). If you use a non-empty ROS '
                "`namespace`, consider setting this to '<namespace>/tesseract_monitor'."
            ),
        ),
        DeclareLaunchArgument(
            "tesseract_joint_state_topic",
            default_value="",
            description=(
                "Joint states topic name for the Tesseract monitor. If empty, "
                "the monitor uses its internal default (typically '/joint_states')."
            ),
        ),
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="rlc_moveit_config",
            description="Package that provides the MoveIt config YAML files.",
        ),
        DeclareLaunchArgument(
            "kinematics_yaml",
            default_value="config/fr3/kinematics.yaml",
            description="Path to kinematics YAML relative to moveit_config_package.",
        ),
        DeclareLaunchArgument(
            "joint_limits_yaml",
            default_value="config/fr3/joint_limits.yaml",
            description="Path to joint limits YAML relative to moveit_config_package.",
        ),
        DeclareLaunchArgument(
            "controllers_yaml",
            default_value="config/fr3/controllers.yaml",
            description="Path to MoveIt controllers YAML relative to moveit_config_package.",
        ),
        DeclareLaunchArgument(
            "ompl_planning_yaml",
            default_value="config/fr3/ompl_planning.yaml",
            description="Path to OMPL planning YAML relative to moveit_config_package.",
        ),
        DeclareLaunchArgument(
            "trajopt_planning_yaml",
            default_value="config/fr3/trajopt_planning.yaml",
            description=(
                "Path to TrajOpt plugin parameters YAML relative to moveit_config_package."
            ),
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="",
            description="Hostname or IP address of the robot.",
        ),
        DeclareLaunchArgument(
            "load_gripper",
            default_value="true",
            description="Whether to load the gripper (true | false).",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware.",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description=(
                "Fake sensor commands. Only valid when 'use_fake_hardware' is true."
            ),
        ),
    ]

    franka_urdf_xacro = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "fr3",
        "fr3.urdf.xacro",
    )
    robot_description_command = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            franka_urdf_xacro,
            " ros2_control:=false",
            " hand:=",
            load_gripper,
            " robot_type:=fr3",
            " robot_ip:=",
            robot_ip,
            " use_fake_hardware:=",
            use_fake_hardware,
            " fake_sensor_commands:=",
            fake_sensor_commands,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_command, value_type=str)
    }

    franka_srdf_xacro = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "fr3",
        "fr3.srdf.xacro",
    )
    robot_description_semantic_command = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            franka_srdf_xacro,
            " hand:=",
            load_gripper,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_command, value_type=str
        )
    }

    def launch_setup(context, *args, **kwargs):  # noqa: ANN001, ARG001
        """Create actions after resolving launch configurations."""
        planner_value = planner.perform(context)
        moveit_config_package_value = moveit_config_package.perform(context)

        kinematics = load_yaml(
            moveit_config_package_value, kinematics_yaml.perform(context)
        )
        joint_limits = load_yaml(
            moveit_config_package_value, joint_limits_yaml.perform(context)
        )
        controllers = load_yaml(
            moveit_config_package_value, controllers_yaml.perform(context)
        )

        planning_configs = {}
        tesseract_monitor_env = None
        match planner_value:
            case Planner.OMPL:
                planning_configs = {
                    Planner.OMPL: load_yaml(
                        moveit_config_package_value, ompl_planning_yaml.perform(context)
                    )
                }

            case Planner.RLC_TRAJOPT:
                planning_configs = {
                    Planner.OMPL: load_yaml(
                        moveit_config_package_value, ompl_planning_yaml.perform(context)
                    ),
                    Planner.RLC_TRAJOPT: load_yaml(
                        moveit_config_package_value,
                        trajopt_planning_yaml.perform(context),
                    ),
                }

                scene_bridge_params = planning_configs.get(Planner.RLC_TRAJOPT, {}).get(
                    "scene_bridge", {}
                )

                scene_bridge_params["monitor_namespace"] = (
                    tesseract_monitor_namespace.perform(context)
                )

                tesseract_monitor_env = Node(
                    package="tesseract_monitoring",
                    executable="tesseract_monitoring_environment_node",
                    name="tesseract_environment_monitor",
                    namespace=namespace,
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        robot_description,
                        robot_description_semantic,
                        {
                            "monitor_namespace": tesseract_monitor_namespace.perform(
                                context
                            ),
                            "monitored_namespace": "",
                            "joint_state_topic": tesseract_joint_state_topic.perform(
                                context
                            ),
                            "publish_environment": False,
                        },
                    ],
                )
                
        move_group_node = make_move_group_node(
            namespace=namespace,
            use_sim_time=use_sim_time,
            robot_description=robot_description,
            robot_description_semantic=robot_description_semantic,
            kinematics=kinematics,
            joint_limits=joint_limits,
            controllers=controllers,
            planner=planner_value,
            planning_configs=planning_configs,
        )

        actions = [move_group_node]
        if tesseract_monitor_env is not None:
            actions.append(tesseract_monitor_env)

        return actions

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
