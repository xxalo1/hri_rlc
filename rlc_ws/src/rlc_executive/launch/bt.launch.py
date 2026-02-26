import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _resolve_paths(context, *args, **kwargs):
    """
    Resolve config and BehaviorTree paths from launch arguments.

    Rules (applied per argument):
    - If the argument is an absolute path, use it as-is.
    - If the argument contains a directory component, treat it as relative to the
      package share directory.
    - Otherwise, treat it as a bare filename under a package share subdirectory
      (e.g. `config/`, `plugins/`, `bt_trees/`).

    Parameters
    ----------
    context : launch.LaunchContext
        Launch context used to evaluate substitutions.

    Returns
    -------
    list[launch.Action]
        Actions that set resolved launch configurations.
    """
    pkg_share_path = FindPackageShare("rlc_executive").perform(context)

    def resolve_into_subdir(arg_name: str, subdir: str) -> str:
        arg_value = LaunchConfiguration(arg_name).perform(context)
        if os.path.isabs(arg_value):
            return arg_value
        if os.path.dirname(arg_value):
            return os.path.join(pkg_share_path, arg_value)
        return os.path.join(pkg_share_path, subdir, arg_value)

    return [
        SetLaunchConfiguration(
            "executor_config_resolved", resolve_into_subdir("executor_config", "config")
        ),
        SetLaunchConfiguration(
            "profiles_config_resolved",
            resolve_into_subdir("profiles_config", "config"),
        ),
        SetLaunchConfiguration(
            "plugins_resolved", resolve_into_subdir("plugins", "plugins")
        ),
        SetLaunchConfiguration(
            "default_tree_resolved", resolve_into_subdir("default_tree", "bt_trees")
        ),
    ]


def generate_launch_description():
    """
    Launch the BehaviorTree executive with configurable config and tree paths.

    Returns
    -------
    launch.LaunchDescription
        Launch description for the BT executive node.
    """
    executor_config = LaunchConfiguration("executor_config_resolved")
    profiles_config = LaunchConfiguration("profiles_config_resolved")
    plugins = LaunchConfiguration("plugins_resolved")
    default_tree_resolved = LaunchConfiguration("default_tree_resolved")
    log_level = LaunchConfiguration("log_level")
    enable_groot2 = LaunchConfiguration("enable_groot2")
    groot2_port = LaunchConfiguration("groot2_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "executor_config",
                default_value="executor.yaml",
            ),
            DeclareLaunchArgument(
                "profiles_config",
                default_value="planning_profiles.yaml",
            ),
            DeclareLaunchArgument(
                "plugins",
                default_value="bt_plugins.txt",
            ),
            DeclareLaunchArgument("default_tree", default_value="reactive_core.xml"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("enable_groot2", default_value="false"),
            DeclareLaunchArgument("groot2_port", default_value="1667"),
            OpaqueFunction(function=_resolve_paths),
            Node(
                package="rlc_executive",
                executable="rlc_bt_executor",
                name="rlc_executive",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {
                        "executor_config": executor_config,
                        "profiles_config": profiles_config,
                        "plugins": plugins,
                        "default_tree": default_tree_resolved,
                        "enable_groot2": ParameterValue(enable_groot2, value_type=bool),
                        "groot2_port": ParameterValue(groot2_port, value_type=int),
                    },
                    PathJoinSubstitution(
                        [FindPackageShare("rlc_executive"), "config", "target_goal.yaml"]
                    ),
                ],
            ),
        ]
    )
