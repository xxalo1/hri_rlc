"""Helpers for composing MoveIt `move_group` launch actions."""

from __future__ import annotations

from enum import Enum
import os
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml

DEFAULT_REQUEST_ADAPTERS: list[str] = [
    "default_planning_request_adapters/ResolveConstraintFrames",
    "default_planning_request_adapters/ValidateWorkspaceBounds",
    "default_planning_request_adapters/CheckStartStateBounds",
    "default_planning_request_adapters/CheckStartStateCollision",
]

DEFAULT_RESPONSE_ADAPTERS: list[str] = [
    "default_planning_response_adapters/AddTimeOptimalParameterization",
    "default_planning_response_adapters/ValidateSolution",
    "default_planning_response_adapters/DisplayMotionPath",
]


class Planner(Enum):
    """Enum for supported planners."""

    OMPL = "ompl"
    RLC_TRAJOPT = "rlc_trajopt"

_TESSERACT_XMLNS = "https://github.com/tesseract-robotics/tesseract"


def ensure_tesseract_make_convex_attribute(urdf_xml: str, *, make_convex: bool = True) -> str:
    """
    Ensure a URDF has the global `tesseract:make_convex` attribute on `<robot>`.

    Tesseract's URDF parser requires the global attribute `tesseract:make_convex` on
    the `<robot>` element (see `tesseract_urdf::parseURDFString`). The attribute
    indicates whether collision meshes should be globally converted to convex hulls.

    This helper performs a minimal string-level injection on the first `<robot ...>`
    start tag so the attribute name is exactly `tesseract:make_convex` (Tesseract
    queries it by name, not by namespace-aware parsing).

    Parameters
    ----------
    urdf_xml : str
        URDF XML string.
    make_convex : bool, optional
        Value to inject for `tesseract:make_convex` if missing.

    Returns
    -------
    str
        URDF XML string with `xmlns:tesseract` and `tesseract:make_convex` added to
        the `<robot>` start tag if they were missing.

    Raises
    ------
    RuntimeError
        If the `<robot>` element cannot be found in `urdf_xml`.
    """
    robot_open_idx = urdf_xml.find("<robot")
    if robot_open_idx < 0:
        raise RuntimeError("URDF XML does not contain a '<robot' element.")

    robot_tag_end_idx = urdf_xml.find(">", robot_open_idx)
    if robot_tag_end_idx < 0:
        raise RuntimeError(
            "URDF XML contains '<robot' but no closing '>' for the start tag."
        )

    robot_start_tag = urdf_xml[robot_open_idx:robot_tag_end_idx]
    insert = ""

    if "xmlns:tesseract=" not in robot_start_tag:
        insert += f' xmlns:tesseract="{_TESSERACT_XMLNS}"'

    if "tesseract:make_convex=" not in robot_start_tag:
        insert += f' tesseract:make_convex="{"true" if make_convex else "false"}"'

    if not insert:
        return urdf_xml

    return urdf_xml[:robot_tag_end_idx] + insert + urdf_xml[robot_tag_end_idx:]


def load_yaml(package_name: str, relative_path: str) -> dict[str, Any]:
    """
    Load a YAML mapping from a package share path.

    Parameters
    ----------
    package_name : str
        ROS package name used to resolve the share directory.
    relative_path : str
        Path relative to the package share directory, e.g.
        ``config/fr3/kinematics.yaml``.

    Returns
    -------
    dict
        Parsed YAML mapping. Empty if the YAML file is empty.

    Raises
    ------
    RuntimeError
        If the file cannot be read or the YAML does not parse to a mapping.
    """
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    try:
        with open(absolute_path, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}
    except EnvironmentError as ex:
        raise RuntimeError(f"Could not read YAML file: {absolute_path}") from ex
    except yaml.YAMLError as ex:
        raise RuntimeError(f"Could not parse YAML file: {absolute_path}") from ex

    if not isinstance(data, dict):
        raise RuntimeError(
            f"Expected YAML mapping in '{absolute_path}', got '{type(data).__name__}'"
        )

    return data


def planning_plugins_for(planner: Planner) -> list[str]:
    """
    Map a high-level planner selection to MoveIt planner plugin class names.

    Parameters
    ----------
    planner : Planner
        Planner selection.

    Returns
    -------
    list[str]
        Planner plugin class names in the order they are registered.

    Raises
    ------
    ValueError
        If `planner` is unsupported.
    """
    if planner is Planner.OMPL:
        return ["ompl_interface/OMPLPlanner"]
    if planner is Planner.RLC_TRAJOPT:
        return [
            "ompl_interface/OMPLPlanner",
            "rlc_planner/TrajOptPlannerManager",
        ]
    raise ValueError(f"Unsupported planner selection: '{planner}'")


def make_planning_pipeline_config(
    *,
    planner: Planner,
    planning_configs: dict[Planner, Any],
    request_adapters: list[str] | None = None,
    response_adapters: list[str] | None = None,
    start_state_max_bounds_error: float = 0.1,
) -> dict[str, Any]:
    """
    Build the MoveIt planning pipeline parameter mapping for `move_group`.

    Parameters
    ----------
    planner : Planner
        Planner selection enum.
    planning_configs : dict[Planner, Any]
        Planning parameter mappings that will be merged into the `move_group`
        namespace. Values are merged in iteration order, and later keys override
        earlier keys.
    request_adapters : list[str], optional
        Request adapter plugins. If None, uses package defaults.
    response_adapters : list[str], optional
        Response adapter plugins. If None, uses package defaults.
    start_state_max_bounds_error : float, optional
        Max start-state bounds error.

    Returns
    -------
    dict
        A mapping suitable for passing to `launch_ros.actions.Node(parameters=...)`.
    """
    if request_adapters is None:
        request_adapters = list(DEFAULT_REQUEST_ADAPTERS)

    if response_adapters is None:
        response_adapters = list(DEFAULT_RESPONSE_ADAPTERS)

    planning_plugins = planning_plugins_for(planner)

    pipeline = {
        "move_group": {
            "planning_plugins": planning_plugins,
            "request_adapters": request_adapters,
            "response_adapters": response_adapters,
            "start_state_max_bounds_error": start_state_max_bounds_error,
        }
    }

    for config in planning_configs.values():
        pipeline["move_group"].update(config)
    return pipeline


def make_move_group_node(
    *,
    namespace: LaunchConfiguration,
    use_sim_time: bool | LaunchConfiguration = False,
    robot_description: dict[str, Any],
    robot_description_semantic: dict[str, Any],
    kinematics: dict[str, Any],
    joint_limits: dict[str, Any],
    controllers: dict[str, Any],
    planning_configs: dict[Planner, Any],
    planner: Planner,
) -> Node:
    """
    Create the `move_group` node action.

    Parameters
    ----------
    namespace : LaunchConfiguration
        ROS namespace for all nodes.
    use_sim_time : bool or LaunchConfiguration, optional
        Whether to use simulation time for `move_group`.
    robot_description : dict
        `robot_description` parameter mapping.
    robot_description_semantic : dict
        `robot_description_semantic` parameter mapping.
    kinematics : dict
        MoveIt kinematics YAML contents.
    joint_limits : dict
        MoveIt joint limits YAML contents.
    controllers : dict
        MoveIt simple controller manager YAML contents.
    planning_configs : dict[Planner, Any]
        Planning YAML contents.
    planner : Planner
        Planner selection enum.

    Returns
    -------
    Node
        A configured `move_group` node action.
    """
    kinematics_config: dict[str, Any] = {"robot_description_kinematics": kinematics}
    joint_limits_config: dict[str, Any] = {"robot_description_planning": joint_limits}

    planning_pipeline_config = make_planning_pipeline_config(
        planner=planner,
        planning_configs=planning_configs,
    )

    moveit_controllers: dict[str, Any] = {
        "moveit_simple_controller_manager": controllers,
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
    }

    trajectory_execution: dict[str, Any] = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters: dict[str, Any] = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )
