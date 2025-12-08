from __future__ import annotations

from __future__ import annotations

"""Generate a PlotJuggler layout mirroring an exported (GUI-created) layout."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
from xml.dom import minidom


def default_layout_path() -> Path:
    """Prefer installed share directory; fall back to source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("rlc_monitor")) / "config" / "plotjuggler_layout.xml"
    except Exception:
        return Path(__file__).resolve().parent.parent / "config" / "plotjuggler_layout.xml"


def default_topics() -> dict[str, str]:
    """Read topics from rlc_common if available; otherwise use sane defaults."""
    try:
        from rlc_common.endpoints import TOPICS

        return {
            "joint_state": f"/{TOPICS.joint_state.name.lstrip('/')}",
            "effort_cmd": f"/{TOPICS.effort_cmd.name.lstrip('/')}",
            "controller_state": f"/{TOPICS.controller_state.name.lstrip('/')}",
        }
    except Exception:
        return {
            "joint_state": "/sim/gen3/joint_state",
            "effort_cmd": "/ctrl/gen3/effort_cmd",
            "controller_state": "/ctrl/gen3/controller_state",
        }


def build_layout(topics: dict[str, str], joint_names: list[str], layout_name: str, use_header_time: bool) -> ET.Element:
    """Construct a layout similar to one exported from PlotJuggler."""
    root = ET.Element("root")
    ET.SubElement(root, "version").text = "3.6"

    tabbed_widget = ET.SubElement(root, "tabbed_widget", {"name": "Main Window", "parent": "main_window"})

    palette = ["#1f77b4", "#ff7f0e", "#d62728", "#1ac938"]

    for idx, joint in enumerate(joint_names):
        tab = ET.SubElement(tabbed_widget, "Tab", {"containers": "1", "tab_name": joint})
        container = ET.SubElement(tab, "Container")
        splitter = ET.SubElement(container, "DockSplitter", {"sizes": "1", "count": "1", "orientation": "-"})
        dock = ET.SubElement(splitter, "DockArea", {"name": "..."})
        plot = ET.SubElement(dock, "plot", {"flip_y": "false", "mode": "TimeSeries", "flip_x": "false", "style": "Lines"})
        ET.SubElement(plot, "limitY")
        # Sim effort for this joint (JointState names are split out per joint by PlotJuggler)
        ET.SubElement(plot, "curve", {"color": palette[0], "name": f"{topics['joint_state']}/{joint}/effort"})
        # Controller commanded effort (array by index)
        ET.SubElement(plot, "curve", {"color": palette[1], "name": f"{topics['effort_cmd']}/effort[{idx}]"})
        # Controller error e and ed (array by index)
        ET.SubElement(plot, "curve", {"color": palette[2], "name": f"{topics['controller_state']}/error/positions[{idx}]"})
        ET.SubElement(plot, "curve", {"color": palette[3], "name": f"{topics['controller_state']}/error/velocities[{idx}]"})

    ET.SubElement(tabbed_widget, "currentTabIndex", {"index": "0"})

    ET.SubElement(root, "use_relative_time_offset", {"enabled": "1"})

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

    # ROS2 live streamer with the topics we care about
    streamer = ET.SubElement(plugins, "plugin", {"ID": "ROS2 Topic Subscriber"})
    ET.SubElement(streamer, "use_header_stamp", {"value": str(use_header_time).lower()})
    ET.SubElement(streamer, "discard_large_arrays", {"value": "true"})
    ET.SubElement(streamer, "max_array_size", {"value": "100"})
    ET.SubElement(streamer, "boolean_strings_to_number", {"value": "true"})
    ET.SubElement(streamer, "remove_suffix_from_strings", {"value": "true"})
    ET.SubElement(streamer, "selected_topics", {"value": f"{topics['controller_state']};{topics['effort_cmd']};{topics['joint_state']}"})

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


def write_layout(out_path: Path, topics: dict[str, str], joint_count: int, layout_name: str, use_header_time: bool) -> Path:
    joint_names = [f"gen3_joint_{i+1}" for i in range(joint_count)]
    xml_text = serialize_xml(
        build_layout(
            topics=topics,
            joint_names=joint_names,
            layout_name=layout_name,
            use_header_time=use_header_time,
        )
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(xml_text)
    return out_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    topics = default_topics()
    parser = argparse.ArgumentParser(description="Generate a PlotJuggler layout resembling an exported GUI layout.")
    parser.add_argument("--out", "-o", type=Path, default=default_layout_path(), help="Destination layout file.")
    parser.add_argument("--layout-name", default="RLC Telemetry", help="Name of the PlotJuggler layout.")
    parser.add_argument(
        "--joint-count",
        "-n",
        type=int,
        default=7,
        help="Number of joints to create series for.",
    )
    parser.add_argument(
        "--use-header-time",
        action="store_true",
        default=False,
        help="Use ROS header time for PlotJuggler (default: reception time).",
    )
    parser.add_argument(
        "--no-header-time",
        dest="use_header_time",
        action="store_false",
        help="Disable header time and fall back to reception time.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    out_path = write_layout(
        out_path=args.out,
        topics=default_topics(),
        joint_count=args.joint_count,
        layout_name=args.layout_name,
        use_header_time=args.use_header_time,
    )
    print(f"Wrote PlotJuggler layout to: {out_path}")


if __name__ == "__main__":
    main()
