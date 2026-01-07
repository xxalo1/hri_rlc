from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description()-> LaunchDescription:
    pkg_share = get_package_share_directory("rlc_bringup")
    rlc_rbt_desc_share = get_package_share_directory("rlc_robot_descriptions")
    xacro_file = os.path.join(
        rlc_rbt_desc_share, "robots", "kinova_gen3", "urdf", "gen3_7dof_vision_gz.urdf.xacro"
    )
    controllers_yaml = os.path.join(pkg_share, "config", "gen3_gz_controllers.yaml")

    robot_description = {
        "robot_description": Command(
            [
                "ros2",
                "run",
                "xacro",
                "xacro",
                xacro_file,
                f"controllers_yaml:={controllers_yaml}",
            ]
        )
    }

    gz = ExecuteProcess(
        cmd=["gz", "sim", "-r", "empty.sdf"],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", "gen3", "-param", "robot_description"],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["rlc_controller", "--controller-manager", "/controller_manager", "--param-file", controllers_yaml],
    )

    return LaunchDescription([
        gz,
        clock_bridge,
        rsp,
        spawn,
        jsb_spawner,
        ctrl_spawner,
    ])
