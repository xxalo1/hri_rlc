from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Launch FR3 Gazebo bringup plus MoveIt `move_group`.

    Notes
    -----
    This launch composes two existing launch files:

    - `rlc_bringup/launch/fr3_gz.launch.py` for Gazebo Sim, robot spawn, and
      ros2_control controllers.
    - `franka_fr3_moveit_config/launch/move_group.launch.py` for the MoveIt
      planning/execution server.

    `use_sim_time` is applied globally so MoveIt and Gazebo nodes share the same
    clock.

    Returns
    -------
    LaunchDescription
        Combined launch description.
    """

    # Common arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Arguments expected by franka_fr3_moveit_config/launch/move_group.launch.py
    namespace = LaunchConfiguration("namespace")
    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")

    fr3_gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rlc_bringup"), "launch", "fr3_gz.launch.py"]
            )
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("franka_fr3_moveit_config"),
                    "launch",
                    "move_group.launch.py",
                ]
            )
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time if true.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="ROS namespace for the MoveIt move_group node.",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="0.0.0.0",
                description=(
                    "Robot hostname/IP passed into the FR3 xacro. "
                    "For Gazebo this is unused but required by the xacro interface."
                ),
            ),
            DeclareLaunchArgument(
                "load_gripper",
                default_value="false",
                description=(
                    "Whether to include the gripper in MoveIt robot_description. "
                    "Keep this consistent with fr3_gz.launch.py (which uses hand:=false)."
                ),
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description=(
                    "Passed into the FR3 xacro. For Gazebo this is typically true/ignored."
                ),
            ),
            DeclareLaunchArgument(
                "fake_sensor_commands",
                default_value="false",
                description="Passed into the FR3 xacro.",
            ),
            SetParameter(name="use_sim_time", value=use_sim_time),
            fr3_gz_launch,
            move_group_launch,
        ]
    )
