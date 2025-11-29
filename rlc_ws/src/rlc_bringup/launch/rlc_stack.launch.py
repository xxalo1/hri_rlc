from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_viewer = LaunchConfiguration("use_viewer")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_viewer",
                default_value="false",
                description="Use MuJoCo viewer (true) or headless (false)",
            ),
            Node(
                package="rlc_sim",
                executable="gen3_sim_headless",
                name="gen3_sim_headless",
                output="screen",
            ),
            Node(
                package="rlc_sim",
                executable="gen3_sim_viewer",
                name="gen3_sim_viewer",
                output="screen",
                condition=IfCondition(use_viewer),
            ),
            Node(
                package="rlc_controller",
                executable="gen3_controller",
                name="gen3_controller",
                output="screen",
            ),
            Node(
                package="rlc_planner",
                executable="planner_node",
                name="planner_node",
                output="screen",
            ),
            Node(
                package="rlc_planner",
                executable="executor_node",
                name="executor_node",
                output="screen",
            ),
        ]
    )
