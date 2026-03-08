"""
Launch MoveIt Servo as a standalone node, optionally starting it paused.

Design goals
1. Namespace agnostic: all topic and service names should work under any ROS namespace.
2. Consistent with rlc_moveit_config's existing launch style (OpaqueFunction + YAML loading).
3. Start paused by default, then unpause via the standard `~/pause_servo` service when desired.
"""

from __future__ import annotations

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_THIS_DIR = os.path.dirname(__file__)
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from move_group_utils import load_yaml  # noqa: E402


def _normalize_ns(ns: str) -> str:
    ns = (ns or "").strip()
    if ns == "/":
        return ""
    return ns.strip("/")


def _svc_name(ns: str, node_name: str, srv_name: str) -> str:
    n = _normalize_ns(ns)
    if not n:
        return f"/{node_name}/{srv_name}"
    return f"/{n}/{node_name}/{srv_name}"


def generate_launch_description() -> LaunchDescription:
    """
    Launch MoveIt Servo as a standalone node.

    Notes
    -----
    This launch is intended to be composable: callers must provide the URDF/SRDF
    xacro inputs via `urdf_xacro_file` / `srdf_xacro_file` (and optional
    `*_xacro_args`). This launch does not select a robot model by default.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    urdf_xacro_file = LaunchConfiguration("urdf_xacro_file")
    urdf_xacro_args = LaunchConfiguration("urdf_xacro_args")
    srdf_xacro_file = LaunchConfiguration("srdf_xacro_file")
    srdf_xacro_args = LaunchConfiguration("srdf_xacro_args")

    moveit_config_package = LaunchConfiguration("moveit_config_package")
    servo_yaml = LaunchConfiguration("servo_yaml")
    kinematics_yaml = LaunchConfiguration("kinematics_yaml")
    joint_limits_yaml = LaunchConfiguration("joint_limits_yaml")

    planning_group_name = LaunchConfiguration("planning_group_name")
    update_period = LaunchConfiguration("update_period")
    start_paused = LaunchConfiguration("start_paused")
    pause_call_delay = LaunchConfiguration("pause_call_delay")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time if true.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for Servo node.",
        ),
        DeclareLaunchArgument(
            "urdf_xacro_file",
            default_value="",
            description=(
                "URDF xacro file used to generate `robot_description`. This launch does "
                "not assume any robot model; callers must pass this explicitly (absolute "
                "path recommended)."
            ),
        ),
        DeclareLaunchArgument(
            "urdf_xacro_args",
            default_value="",
            description=(
                "Extra xacro args appended verbatim when running `urdf_xacro_file` "
                "(space-separated tokens like `foo:=bar`)."
            ),
        ),
        DeclareLaunchArgument(
            "srdf_xacro_file",
            default_value="",
            description=(
                "SRDF xacro file used to generate `robot_description_semantic`. Callers "
                "must pass this explicitly (absolute path recommended)."
            ),
        ),
        DeclareLaunchArgument(
            "srdf_xacro_args",
            default_value="",
            description=(
                "Extra xacro args appended verbatim when running `srdf_xacro_file` "
                "(space-separated tokens like `foo:=bar`)."
            ),
        ),
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="rlc_moveit_config",
            description="Package that provides the YAML files.",
        ),
        DeclareLaunchArgument(
            "servo_yaml",
            default_value="config/fr3/servo.yaml",
            description="Path to Servo YAML relative to moveit_config_package.",
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
            "planning_group_name",
            default_value="fr3_arm",
            description="Used by the acceleration limiting smoothing plugin.",
        ),
        DeclareLaunchArgument(
            "update_period",
            default_value="0.01",
            description="Used by the acceleration limiting smoothing plugin.",
        ),
        DeclareLaunchArgument(
            "start_paused",
            default_value="true",
            description="If true, call `pause_servo` once after startup.",
        ),
        DeclareLaunchArgument(
            "pause_call_delay",
            default_value="1.0",
            description="Seconds to wait before calling pause_servo on startup.",
        ),
    ]

    def launch_setup(context, *args, **kwargs):  # noqa: ANN001, ARG001
        moveit_config_package_value = moveit_config_package.perform(context)

        urdf_xacro_file_value = urdf_xacro_file.perform(context).strip()
        srdf_xacro_file_value = srdf_xacro_file.perform(context).strip()
        if not urdf_xacro_file_value or not srdf_xacro_file_value:
            raise RuntimeError(
                "Both `urdf_xacro_file` and `srdf_xacro_file` must be provided. "
            )
        if not os.path.exists(urdf_xacro_file_value):
            raise RuntimeError(f"URDF xacro file not found: '{urdf_xacro_file_value}'")
        if not os.path.exists(srdf_xacro_file_value):
            raise RuntimeError(f"SRDF xacro file not found: '{srdf_xacro_file_value}'")

        urdf_xacro_args_value = urdf_xacro_args.perform(context).strip()
        srdf_xacro_args_value = srdf_xacro_args.perform(context).strip()

        robot_description_cmd = [
            FindExecutable(name="xacro"),
            " ",
            urdf_xacro_file_value,
        ]
        if urdf_xacro_args_value:
            robot_description_cmd += [" ", urdf_xacro_args_value]
        robot_description_command = Command(robot_description_cmd)
        robot_description = {
            "robot_description": ParameterValue(
                robot_description_command, value_type=str
            )
        }

        robot_description_semantic_cmd = [
            FindExecutable(name="xacro"),
            " ",
            srdf_xacro_file_value,
        ]
        if srdf_xacro_args_value:
            robot_description_semantic_cmd += [" ", srdf_xacro_args_value]
        robot_description_semantic_command = Command(robot_description_semantic_cmd)
        robot_description_semantic = {
            "robot_description_semantic": ParameterValue(
                robot_description_semantic_command, value_type=str
            )
        }

        kinematics = load_yaml(
            moveit_config_package_value, kinematics_yaml.perform(context)
        )
        joint_limits = load_yaml(
            moveit_config_package_value, joint_limits_yaml.perform(context)
        )

        servo_yaml_rel = servo_yaml.perform(context)
        servo_yaml_abs = os.path.join(
            get_package_share_directory(moveit_config_package_value), servo_yaml_rel
        )
        if not os.path.exists(servo_yaml_abs):
            raise RuntimeError(
                f"Servo YAML not found: '{servo_yaml_abs}' "
                f"(from servo_yaml:='{servo_yaml_rel}', moveit_config_package:='{moveit_config_package_value}')"
            )

        kinematics_config = {"robot_description_kinematics": kinematics}
        joint_limits_config = {"robot_description_planning": joint_limits}

        servo_node = Node(
            package="moveit_servo",
            executable="servo_node",
            name="servo_node",
            namespace=namespace,
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                # Servo params file (ROS2 parameter YAML).
                servo_yaml_abs,
                # Params used by the acceleration limiting smoothing plugin.
                {"update_period": float(update_period.perform(context))},
                {"planning_group_name": planning_group_name.perform(context)},
                # Robot model params.
                robot_description,
                robot_description_semantic,
                kinematics_config,
                joint_limits_config,
            ],
        )

        actions = [servo_node]

        start_paused_value = start_paused.perform(context).lower() in (
            "1",
            "true",
            "yes",
        )
        if start_paused_value:
            ns_value = namespace.perform(context)
            pause_service = _svc_name(ns_value, "servo_node", "pause_servo")

            pause_cmd = ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    pause_service,
                    "std_srvs/srv/SetBool",
                    "{data: true}",
                ],
                output="screen",
            )

            actions.append(
                TimerAction(
                    period=float(pause_call_delay.perform(context)),
                    actions=[pause_cmd],
                )
            )

        return actions

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
