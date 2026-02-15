from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Launch FR3 in Gazebo Sim and MoveIt `move_group`.

    Notes
    -----
    This is a composition launch that includes:
    - `rlc_bringup/launch/fr3_gz_control.launch.py` (Gazebo + ros2_control + controllers)
    - `rlc_moveit_config/launch/move_group.launch.py` (MoveIt)
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    xacro_file = LaunchConfiguration("xacro_file")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    load_gripper = LaunchConfiguration("load_gripper")
    ee_id = LaunchConfiguration("ee_id")
    controller_manager = LaunchConfiguration("controller_manager")
    arm_controller = LaunchConfiguration("arm_controller")
    gripper_controller = LaunchConfiguration("gripper_controller")

    moveit_namespace = LaunchConfiguration("namespace")
    planner = LaunchConfiguration("planner")

    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_kinematics_yaml = LaunchConfiguration("kinematics_yaml")
    moveit_joint_limits_yaml = LaunchConfiguration("joint_limits_yaml")
    moveit_controllers_yaml = LaunchConfiguration("moveit_controllers_yaml")
    moveit_ompl_planning_yaml = LaunchConfiguration("ompl_planning_yaml")
    moveit_trajopt_planning_yaml = LaunchConfiguration("trajopt_planning_yaml")
    tesseract_monitor_namespace = LaunchConfiguration("tesseract_monitor_namespace")
    tesseract_joint_state_topic = LaunchConfiguration("tesseract_joint_state_topic")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    gz_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rlc_bringup"),
                    "launch",
                    "fr3_gz_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
            "robot_name": robot_name,
            "xacro_file": xacro_file,
            "controllers_yaml": controllers_yaml,
            "load_gripper": load_gripper,
            "ee_id": ee_id,
            "controller_manager": controller_manager,
            "arm_controller": arm_controller,
            "gripper_controller": gripper_controller,
        }.items(),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rlc_moveit_config"),
                    "launch",
                    "move_group.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": moveit_namespace,
            "planner": planner,
            "moveit_config_package": moveit_config_package,
            "kinematics_yaml": moveit_kinematics_yaml,
            "joint_limits_yaml": moveit_joint_limits_yaml,
            "moveit_controllers_yaml": moveit_controllers_yaml,
            "ompl_planning_yaml": moveit_ompl_planning_yaml,
            "trajopt_planning_yaml": moveit_trajopt_planning_yaml,
            "tesseract_monitor_namespace": tesseract_monitor_namespace,
            "tesseract_joint_state_topic": tesseract_joint_state_topic,
            "load_gripper": load_gripper,
            "use_rviz": use_rviz,
            "rviz_config": rviz_config,
        }.items(),
    )

    return LaunchDescription(
        [
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
                default_value="fr3",
                description="Entity name to spawn in Gazebo.",
            ),
            DeclareLaunchArgument(
                "xacro_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("franka_description"),
                        "robots",
                        "fr3",
                        "fr3.urdf.xacro",
                    ]
                ),
                description="Robot URDF Xacro file.",
            ),
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rlc_bringup"),
                        "config",
                        "fr3_gz_controllers.yaml",
                    ]
                ),
                description="ros2_control controller manager YAML.",
            ),
            DeclareLaunchArgument(
                "load_gripper",
                default_value="false",
                description="Whether to include the gripper in the Gazebo model.",
            ),
            DeclareLaunchArgument(
                "ee_id",
                default_value="franka_hand",
                description="End-effector id for FR3 xacros (e.g., franka_hand, none).",
            ),
            DeclareLaunchArgument(
                "controller_manager",
                default_value="/controller_manager",
                description="Controller manager node name or namespace.",
            ),
            DeclareLaunchArgument(
                "arm_controller",
                default_value="arm_controller",
                description="Controller name to spawn for the arm.",
            ),
            DeclareLaunchArgument(
                "gripper_controller",
                default_value="gripper_controller",
                description="Controller name to spawn for the gripper.",
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
                default_value="config/fr3/controllers_gz.yaml",
                description="MoveIt controller manager YAML (Gazebo).",
            ),
            DeclareLaunchArgument(
                "ompl_planning_yaml",
                default_value="config/fr3/ompl_planning.yaml",
                description="Path to OMPL planning YAML relative to moveit_config_package.",
            ),
            DeclareLaunchArgument(
                "trajopt_planning_yaml",
                default_value="config/fr3/trajopt_planning.yaml",
                description="Path to TrajOpt parameters YAML relative to moveit_config_package.",
            ),
            DeclareLaunchArgument(
                "tesseract_monitor_namespace",
                default_value="tesseract_monitor",
                description=(
                    "Tesseract monitor namespace used for services/topics (e.g., "
                    '"/<ns>/get_tesseract_information").'
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
            gz_control_launch,
            move_group_launch,
        ]
    )
