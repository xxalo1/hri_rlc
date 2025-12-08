from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_viewer = LaunchConfiguration("use_viewer")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_viewer",
                default_value="false",
                description="Use MuJoCo viewer (true) or headless (false)",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="INFO",
                description="Global ROS logger level: DEBUG, INFO, WARN, ERROR, FATAL",
            ),
            # Ensure rcutils honors per-node log-level
            SetEnvironmentVariable(
                name="RCUTILS_LOGGING_SEVERITY_THRESHOLD",
                value=log_level,
            ),
            Node(
                package="rlc_sim",
                executable="gen3_sim_headless",
                name="gen3_sim_headless",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="rlc_sim",
                executable="gen3_sim_viewer",
                name="gen3_sim_viewer",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                condition=IfCondition(use_viewer),
            ),
            Node(
                package="rlc_controller",
                executable="gen3_controller",
                name="gen3_controller",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="rlc_planner",
                executable="planner_node",
                name="planner_node",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="rlc_planner",
                executable="executor_node",
                name="executor_node",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="rlc_monitor",
                executable="monitor_node",
                name="rlc_monitor",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )
