import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_THIS_DIR = os.path.dirname(__file__)
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from move_group_utils import (
    load_yaml,
    make_move_group_node,
    make_planning_pipeline_config,
    Planner,
    ensure_tesseract_make_convex_attribute,
)


def generate_launch_description() -> LaunchDescription:
    """
    Launch the MoveIt `move_group` node (and optional Tesseract monitor / RViz).

    Parameters are declared as launch arguments so another launch file can
    include this launch file and override:
    - which MoveIt config YAML files to use
    - which planner pipeline to enable (default: OMPL)
    - how `robot_description` / `robot_description_semantic` are generated (URDF/SRDF xacro)

    Notes
    -----
    When `planner:=rlc_trajopt`, this launch also starts:
    - a `tesseract_monitoring_environment_node` for hosting the Tesseract environment, and
    - an `rlc_scene_bridge_node` for synchronizing MoveIt's planning scene into that
      environment.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    planner = LaunchConfiguration("planner")
    tesseract_monitor_namespace = LaunchConfiguration("tesseract_monitor_namespace")
    tesseract_joint_state_topic = LaunchConfiguration("tesseract_joint_state_topic")

    urdf_xacro_file = LaunchConfiguration("urdf_xacro_file")
    urdf_xacro_args = LaunchConfiguration("urdf_xacro_args")
    srdf_xacro_file = LaunchConfiguration("srdf_xacro_file")
    srdf_xacro_args = LaunchConfiguration("srdf_xacro_args")

    moveit_config_package = LaunchConfiguration("moveit_config_package")
    kinematics_yaml = LaunchConfiguration("kinematics_yaml")
    joint_limits_yaml = LaunchConfiguration("joint_limits_yaml")
    moveit_controllers_yaml = LaunchConfiguration("moveit_controllers_yaml")
    ompl_planning_yaml = LaunchConfiguration("ompl_planning_yaml")
    trajopt_planning_yaml = LaunchConfiguration("trajopt_planning_yaml")
    scene_bridge_yaml = LaunchConfiguration("scene_bridge_yaml")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

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
            "moveit_controllers_yaml",
            default_value="config/fr3/controllers.yaml",
            description=(
                "Path to MoveIt controllers YAML relative to `moveit_config_package`."
            ),
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
            "scene_bridge_yaml",
            default_value="config/fr3/scene_bridge.yaml",
            description=(
                "Path to rlc_scene_bridge node parameters YAML relative to "
                "moveit_config_package."
            ),
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz if true.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="rviz/moveit.rviz",
            description=(
                "RViz config file path. If relative, it is resolved relative to "
                "`moveit_config_package`."
            ),
        ),
    ]

    def launch_setup(context, *args, **kwargs):  # noqa: ANN001, ARG001
        """Create actions after resolving launch configurations."""
        planner_value = planner.perform(context)
        try:
            planner_enum = Planner(planner_value)
        except ValueError as ex:
            raise RuntimeError(
                f"Invalid planner selection: '{planner_value}'. "
                f"Valid options are: {[p.value for p in Planner]}"
            ) from ex

        moveit_config_package_value = moveit_config_package.perform(context)

        urdf_xacro_file_value = urdf_xacro_file.perform(context).strip()
        srdf_xacro_file_value = srdf_xacro_file.perform(context).strip()
        if not urdf_xacro_file_value or not srdf_xacro_file_value:
            raise RuntimeError(
                "Both `urdf_xacro_file` and `srdf_xacro_file` must be provided. "
                "This launch file does not select a robot model by default."
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
        controllers = load_yaml(
            moveit_config_package_value, moveit_controllers_yaml.perform(context)
        )

        planning_configs = {}
        tesseract_monitor_env = None
        scene_bridge_node = None

        match planner_enum:
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

                trajopt_params = planning_configs[Planner.RLC_TRAJOPT]
                monitor_params = trajopt_params.setdefault("monitor", {})
                if not isinstance(monitor_params, dict):
                    raise RuntimeError(
                        "Expected 'monitor' to be a mapping in trajopt_planning_yaml."
                    )
                monitor_params["monitor_namespace"] = (
                    tesseract_monitor_namespace.perform(context)
                )

                scene_bridge_params = load_yaml(
                    moveit_config_package_value, scene_bridge_yaml.perform(context)
                )
                scene_bridge_section = scene_bridge_params.setdefault("scene_bridge", {})
                if not isinstance(scene_bridge_section, dict):
                    raise RuntimeError(
                        "Expected 'scene_bridge' to be a mapping in scene_bridge_yaml."
                    )
                scene_bridge_section["monitor_namespace"] = (
                    tesseract_monitor_namespace.perform(context)
                )

                tesseract_urdf_xml = robot_description_command.perform(context)
                tesseract_robot_description = {
                    "robot_description": ensure_tesseract_make_convex_attribute(
                        tesseract_urdf_xml, make_convex=True
                    )
                }

                tesseract_monitor_env = Node(
                    package="tesseract_monitoring",
                    executable="tesseract_monitoring_environment_node",
                    namespace=namespace,
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        tesseract_robot_description,
                        robot_description_semantic,
                        {
                            "monitor_namespace": tesseract_monitor_namespace,
                            "monitored_namespace": "",
                            "joint_state_topic": tesseract_joint_state_topic,
                            "publish_environment": False,
                        },
                    ],
                )

                scene_bridge_node = Node(
                    package="rlc_scene_bridge",
                    executable="rlc_scene_bridge_node",
                    namespace=namespace,
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        scene_bridge_params,
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
            planner=planner_enum,
            planning_configs=planning_configs,
        )

        rviz_config_value = rviz_config.perform(context)
        if os.path.isabs(rviz_config_value):
            rviz_config_path = rviz_config_value
        else:
            rviz_config_path = os.path.join(
                get_package_share_directory(moveit_config_package_value),
                rviz_config_value,
            )
        if not os.path.exists(rviz_config_path):
            raise RuntimeError(
                f"RViz config file not found: '{rviz_config_path}' "
                f"(from rviz_config:='{rviz_config_value}', "
                f"moveit_config_package:='{moveit_config_package_value}')"
            )

        rviz_planning_pipeline_config = make_planning_pipeline_config(
            planner=planner_enum,
            planning_configs=planning_configs,
        )
        rviz_kinematics_config = {"robot_description_kinematics": kinematics}

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=namespace,
            output="log",
            arguments=["-d", rviz_config_path],
            parameters=[
                {"use_sim_time": use_sim_time},
                robot_description,
                robot_description_semantic,
                rviz_planning_pipeline_config,
                rviz_kinematics_config,
            ],
            condition=IfCondition(use_rviz),
        )

        actions = []
        if tesseract_monitor_env is not None:
            actions.append(tesseract_monitor_env)
        if scene_bridge_node is not None:
            actions.append(scene_bridge_node)
        actions.append(move_group_node)
        actions.append(rviz_node)

        return actions

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
