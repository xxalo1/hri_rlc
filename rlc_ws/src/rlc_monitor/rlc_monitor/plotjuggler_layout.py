from __future__ import annotations

"""Programmatic PlotJuggler layout builder (no CLI parsing)."""

from pathlib import Path
import xml.etree.ElementTree as ET
from xml.dom import minidom

from rlc_common.endpoints import TOPICS as RLC_TOPICS

# Static topic paths (always available in this package).
TOPIC_PATHS = {
    "joint_state": f"/{RLC_TOPICS.joint_state.name.lstrip('/')}",
    "effort_cmd": f"/{RLC_TOPICS.effort_cmd.name.lstrip('/')}",
    "controller_state": f"/{RLC_TOPICS.controller_state.name.lstrip('/')}",
    "joint_state_action": f"/{RLC_TOPICS.joint_state_action.name.lstrip('/')}",
}


def default_layout_path() -> Path:
    """Prefer installed share directory; fall back to source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("rlc_monitor")) / "config" / "plotjuggler_layout.xml"
    except Exception:
        return Path(__file__).resolve().parent.parent / "config" / "plotjuggler_layout.xml"


def joint_effort_error_tabs(joint_names: list[str]) -> list[dict]:
    """
    Build per-joint tabs with sim effort, commanded effort, and controller e/ed.

    Returns a list of tab dicts:
      {"name": <tab name>, "curves": [{"name": <series>, "color": <hex>}, ...]}
    """
    palette = ["#ff0000", "#001eff", "#002f87", "#8a0000", "#007d21"]
    tabs: list[dict] = []
    for idx, joint in enumerate(joint_names):
        curves = [
            {"name": f"{TOPIC_PATHS['joint_state_action']}/position[{idx}]", "color": palette[0]},
            {"name": f"{TOPIC_PATHS['controller_state']}/feedback/positions[{idx}]", "color": palette[1]}, # ed
            {"name": f"{TOPIC_PATHS['joint_state_action']}/action[{idx}]", "color": palette[2]},
            {"name": f"{TOPIC_PATHS['joint_state_action']}/action_baseline[{idx}]", "color": palette[3]},
            {"name": f"{TOPIC_PATHS['effort_cmd']}/effort[{idx}]", "color": palette[4]},
        ]
        tabs.append({"name": joint, "curves": curves})
    return tabs


def build_layout_from_tabs(tabs: list[dict], layout_name: str, use_header_time: bool) -> ET.Element:
    """
    Build a PlotJuggler layout from a list of tab dicts.

    Each tab dict must contain:
      - name: str
      - curves: list of dicts with keys {"name": series_path, "color": hex_color}
    """
    root = ET.Element("root")
    ET.SubElement(root, "version").text = "3.6"

    tabbed_widget = ET.SubElement(root, "tabbed_widget", {"name": "Main Window", "parent": "main_window"})
    for tab_def in tabs:
        tab = ET.SubElement(tabbed_widget, "Tab", {"containers": "1", "tab_name": tab_def["name"]})
        container = ET.SubElement(tab, "Container")
        splitter = ET.SubElement(container, "DockSplitter", {"sizes": "1", "count": "1", "orientation": "-"})
        dock = ET.SubElement(splitter, "DockArea", {"name": "..."})
        plot = ET.SubElement(dock, "plot", {"flip_y": "false", "mode": "TimeSeries", "flip_x": "false", "style": "Lines"})
        ET.SubElement(plot, "limitY")
        for curve in tab_def["curves"]:
            ET.SubElement(plot, "curve", {"color": curve["color"], "name": curve["name"]})

    ET.SubElement(tabbed_widget, "currentTabIndex", {"index": "0"})
    ET.SubElement(root, "use_relative_time_offset", {"enabled": "1"})

    # Plugins configuration mirrors the exported layout to keep ROS2 streaming happy.
    plugins = ET.SubElement(root, "Plugins")
    ET.SubElement(plugins, "plugin", {"ID": "DataLoad CSV", "delimiter": "0", "time_axis": ""})
    ET.SubElement(plugins, "plugin", {"ID": "DataLoad MCAP"})
    dl_ros2 = ET.SubElement(plugins, "plugin", {"ID": "DataLoad ROS2 bags"})
    ET.SubElement(dl_ros2, "use_header_stamp", {"value": str(use_header_time).lower()})
    ET.SubElement(dl_ros2, "discard_large_arrays", {"value": "true"})
    ET.SubElement(dl_ros2, "max_array_size", {"value": "100"})
    ET.SubElement(dl_ros2, "boolean_strings_to_number", {"value": "true"})
    ET.SubElement(dl_ros2, "remove_suffix_from_strings", {"value": "true"})
    ET.SubElement(dl_ros2, "selected_topics", {"value": ""})

    streamer = ET.SubElement(plugins, "plugin", {"ID": "ROS2 Topic Subscriber"})
    ET.SubElement(streamer, "use_header_stamp", {"value": str(use_header_time).lower()})
    ET.SubElement(streamer, "discard_large_arrays", {"value": "true"})
    ET.SubElement(streamer, "max_array_size", {"value": "100"})
    ET.SubElement(streamer, "boolean_strings_to_number", {"value": "true"})
    ET.SubElement(streamer, "remove_suffix_from_strings", {"value": "true"})
    ET.SubElement(
        streamer,
        "selected_topics",
        {"value": f"{TOPIC_PATHS['controller_state']};{TOPIC_PATHS['effort_cmd']};{TOPIC_PATHS['joint_state']}"},
    )

    ET.SubElement(plugins, "plugin", {"ID": "UDP Server"})
    ET.SubElement(plugins, "plugin", {"ID": "WebSocket Server"})
    ET.SubElement(plugins, "plugin", {"ID": "ZMQ Subscriber"})
    ET.SubElement(plugins, "plugin", {"ID": "Fast Fourier Transform"})
    ET.SubElement(plugins, "plugin", {"ID": "Quaternion to RPY"})
    ET.SubElement(plugins, "plugin", {"ID": "Reactive Script Editor", "library": "", "scripts": ""})
    ET.SubElement(plugins, "plugin", {"ID": "CSV Exporter"})
    ET.SubElement(plugins, "plugin", {"ID": "ROS2 Topic Re-Publisher"})

    ET.SubElement(root, "previouslyLoaded_Datafiles")
    ET.SubElement(root, "previouslyLoaded_Streamer", {"name": "ROS2 Topic Subscriber"})
    ET.SubElement(root, "customMathEquations")
    ET.SubElement(root, "snippets")

    return root


def serialize_xml(elem: ET.Element) -> str:
    """Return pretty-printed XML with declaration."""
    rough = ET.tostring(elem, encoding="utf-8")
    parsed = minidom.parseString(rough)
    return parsed.toprettyxml(indent="  ", encoding="UTF-8").decode("utf-8")


def write_layout(out_path: Path, tabs: list[dict], layout_name: str = "RLC Telemetry", use_header_time: bool = False) -> Path:
    """
    Write the layout file from tab definitions.

    tabs: list of {"name": str, "curves": [{"name": str, "color": str}, ...]}
    """
    xml_text = serialize_xml(build_layout_from_tabs(tabs=tabs, layout_name=layout_name, use_header_time=use_header_time))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(xml_text)
    return out_path


def main() -> None:
    """Generate the default per-joint effort/error layout."""
    joint_names = [f"gen3_joint_{i+1}" for i in range(7)]
    tabs = joint_effort_error_tabs(joint_names)
    out_path = write_layout(default_layout_path(), tabs, layout_name="RLC Telemetry", use_header_time=False)
    print(f"Wrote PlotJuggler layout to: {out_path}")


if __name__ == "__main__":
    main()
