from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    start_stack = LaunchConfiguration("start_stack")
    start_plotjuggler = LaunchConfiguration("start_plotjuggler")
    use_viewer = LaunchConfiguration("use_viewer")
    log_level = LaunchConfiguration("log_level")
    layout_name = LaunchConfiguration("layout_name")

    layout_path = PathJoinSubstitution(
        [FindPackageShare("rlc_monitor"), "config", "plotjuggler_layout.xml"]
    )

    generate_layout = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            "-m",
            "rlc_monitor.plotjuggler_layout",
            "--out",
            layout_path,
            "--layout-name",
            layout_name,
        ],
        name="plotjuggler_layout_gen",
        output="screen",
        condition=IfCondition(start_plotjuggler),
    )

    start_plotjuggler = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "run",
            "plotjuggler",
            "plotjuggler",
            "--layout",
            layout_path,
        ],
        name="plotjuggler_gui",
        output="screen",
        condition=IfCondition(start_plotjuggler),
    )

    include_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rlc_bringup"), "launch", "rlc_stack.launch.py"]
            )
        ),
        condition=IfCondition(start_stack),
        launch_arguments={
            "use_viewer": use_viewer,
            "log_level": log_level,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_stack",
                default_value="true",
                description="Start the RLC stack.",
            ),
            DeclareLaunchArgument(
                "start_plotjuggler",
                default_value="true",
                description="Start PlotJuggler after launching the stack.",
            ),
            DeclareLaunchArgument(
                "use_viewer",
                default_value="true",
                description="Use MuJoCo viewer for the sim node.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="INFO",
                description="Global ROS logger level.",
            ),
            DeclareLaunchArgument(
                "layout_name",
                default_value="RLC Telemetry",
                description="Display name for the PlotJuggler layout.",
            ),
            include_stack,
            generate_layout,
            # Small delay to ensure the layout file exists before launching PlotJuggler.
            TimerAction(period=0.5, actions=[start_plotjuggler]),
        ]
    )
