from __future__ import annotations

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Launch Gazebo Sim with a Kinova Gen3 model and ros2_control controllers.

    Notes
    -----
    Amends Gazebo search paths so `gz_ros2_control` and `kortex_description`
    assets are discoverable in overlay workspaces.

    Returns
    -------
    LaunchDescription
        Launch description for Gazebo + robot_state_publisher + controller spawners.
    """

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    xacro_file = LaunchConfiguration("xacro_file")
    controllers_yaml = LaunchConfiguration("controllers_yaml")

    gz_ros2_control_system_plugins = PathJoinSubstitution(
        [FindPackagePrefix("gz_ros2_control"), "lib"]
    )
    gz_kortex_resources = PathJoinSubstitution(
        [FindPackagePrefix("kortex_description"), "share"]
    )

    default_xacro = PathJoinSubstitution(
        [
            FindPackageShare("rlc_robot_descriptions"),
            "robots",
            "kinova_gen3",
            "urdf",
            "gen3_7dof_vision_gz.urdf.xacro",
        ]
    )
    default_controllers = PathJoinSubstitution(
        [FindPackageShare("rlc_bringup"), "config", "gen3_gz_controllers.yaml"]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    xacro_file,
                    " ",
                ]
            ),
            value_type=str,
        )
    }

    gz = ExecuteProcess(
        cmd=[FindExecutable(name="gz"), "sim", "-r", world],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", robot_name, "-param", "robot_description"],
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "realtime_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_yaml,
            "--controller-manager-timeout",
            "60",
        ],
    )

    start_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])
    )
    start_ctrl_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[ctrl_spawner])
    )

    return LaunchDescription(
        [
            AppendEnvironmentVariable(
                name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                value=gz_ros2_control_system_plugins,
                prepend=True,
            ),
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=gz_kortex_resources,
                prepend=True,
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time if true.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="empty.sdf",
                description="Gazebo world SDF (path or resource name).",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="gen3",
                description="Entity name to spawn in Gazebo.",
            ),
            DeclareLaunchArgument(
                "xacro_file",
                default_value=default_xacro,
                description="Robot URDF Xacro file.",
            ),
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value=default_controllers,
                description="ros2_control controller manager YAML.",
            ),
            gz,
            clock_bridge,
            rsp,
            spawn,
            start_jsb_after_spawn,
            start_ctrl_after_jsb,
        ]
    )
