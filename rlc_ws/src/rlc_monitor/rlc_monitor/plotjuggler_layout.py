from __future__ import annotations

"""
Generate a PlotJuggler layout for the RLC stack.

Defaults align with topics from ``rlc_common.TOPICS`` and target a 7-DOF arm,
but everything can be overridden via CLI flags. The script is intentionally
structured so tabs/plots can be extended without touching the launch files.
"""

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET
from xml.dom import minidom


@dataclass(frozen=True)
class Series:
    topic: str
    field: str
    label: str
    color: str


@dataclass(frozen=True)
class Plot:
    name: str
    series: list[Series]


@dataclass(frozen=True)
class Tab:
    name: str
    plots: list[Plot]


def default_layout_path() -> Path:
    """
    Resolve the output path for the layout.

    Prefers the installed share directory; falls back to the source tree config.
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("rlc_monitor")) / "config" / "plotjuggler_layout.xml"
    except Exception:
        return Path(__file__).resolve().parent.parent / "config" / "plotjuggler_layout.xml"


def default_topics() -> dict[str, str]:
    """
    Fetch topic names from rlc_common, or fall back to the expected defaults.

    The fallback keeps this generator usable even when ROS message deps are
    unavailable in the current environment.
    """
    try:
        from rlc_common.endpoints import TOPICS

        return {
            "sim_tau": TOPICS.joint_state.name,
            "cmd_tau": TOPICS.effort_cmd.name,
            "ctrl_state": TOPICS.controller_state.name,
        }
    except Exception:
        return {
            "sim_tau": "sim/gen3/joint_state",
            "cmd_tau": "ctrl/gen3/effort_cmd",
            "ctrl_state": "ctrl/gen3/controller_state",
        }


def torque_tab(sim_tau_topic: str, cmd_tau_topic: str, joint_count: int) -> Tab:
    plots = [
        Plot(
            name=f"joint{idx}_tau",
            series=[
                Series(
                    topic=sim_tau_topic,
                    field=f"effort[{idx}]",
                    label=f"tau_sim[{idx}]",
                    color="#1f77b4",
                ),
                Series(
                    topic=cmd_tau_topic,
                    field=f"effort[{idx}]",
                    label=f"tau_cmd[{idx}]",
                    color="#ff7f0e",
                ),
            ],
        )
        for idx in range(joint_count)
    ]
    return Tab(name="Torques", plots=plots)


def controller_state_tab(ctrl_state_topic: str, joint_count: int) -> Tab:
    plots = [
        Plot(
            name=f"joint{idx}_state",
            series=[
                Series(
                    topic=ctrl_state_topic,
                    field=f"error.positions[{idx}]",
                    label=f"e[{idx}]",
                    color="#d62728",
                ),
                Series(
                    topic=ctrl_state_topic,
                    field=f"error.velocities[{idx}]",
                    label=f"ed[{idx}]",
                    color="#9467bd",
                ),
                Series(
                    topic=ctrl_state_topic,
                    field=f"feedback.velocities[{idx}]",
                    label=f"v[{idx}]",
                    color="#2ca02c",
                ),
            ],
        )
        for idx in range(joint_count)
    ]
    return Tab(name="Controller State", plots=plots)


def build_layout(
    tabs: Iterable[Tab],
    layout_name: str,
    *,
    use_header_time: bool = True,
) -> ET.Element:
    """Construct the PlotJuggler layout XML tree."""
    # PlotJuggler expects the top-level element to be named "root".
    root = ET.Element("root")
    ET.SubElement(root, "version").text = "3.6"

    data_sources = ET.SubElement(root, "data_sources")
    ros2_live = ET.SubElement(data_sources, "ROS2Live")
    ET.SubElement(ros2_live, "use_header_time").text = str(use_header_time).lower()
    ET.SubElement(ros2_live, "qos_reliable").text = "true"
    ET.SubElement(ros2_live, "qos_durability").text = "volatile"

    layouts = ET.SubElement(root, "layouts")
    layout = ET.SubElement(layouts, "layout", {"name": layout_name})

    for tab in tabs:
        tab_el = ET.SubElement(layout, "tab", {"name": tab.name})
        for plot in tab.plots:
            plot_el = ET.SubElement(tab_el, "plot", {"name": plot.name})
            for series in plot.series:
                ET.SubElement(
                    plot_el,
                    "series",
                    {
                        "topic": series.topic,
                        "field": series.field,
                        "label": series.label,
                        "color": series.color,
                    },
                )

    return root


def serialize_xml(elem: ET.Element) -> str:
    """Return pretty-printed XML with declaration."""
    rough = ET.tostring(elem, encoding="utf-8")
    parsed = minidom.parseString(rough)
    return parsed.toprettyxml(indent="  ", encoding="UTF-8").decode("utf-8")


def default_tabs(
    sim_tau_topic: str,
    cmd_tau_topic: str,
    ctrl_state_topic: str,
    joint_count: int,
) -> list[Tab]:
    if joint_count < 1:
        raise ValueError("joint_count must be positive")
    return [
        torque_tab(sim_tau_topic, cmd_tau_topic, joint_count),
        controller_state_tab(ctrl_state_topic, joint_count),
    ]


def write_layout(out_path: Path, tabs: Iterable[Tab], layout_name: str, use_header_time: bool) -> Path:
    xml_text = serialize_xml(
        build_layout(
            tabs,
            layout_name,
            use_header_time=use_header_time,
        )
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(xml_text)
    return out_path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    default_out = default_layout_path()
    topics = default_topics()
    parser = argparse.ArgumentParser(description="Generate PlotJuggler layout XML for the RLC stack.")
    parser.add_argument("--out", "-o", type=Path, default=default_out, help="Destination layout file.")
    parser.add_argument("--layout-name", default="RLC Telemetry", help="Name of the PlotJuggler layout.")
    parser.add_argument(
        "--joint-count",
        "-n",
        type=int,
        default=7,
        help="Number of joints to create plots for.",
    )
    parser.add_argument(
        "--sim-tau-topic",
        default=topics["sim_tau"],
        help="Topic providing simulated joint efforts.",
    )
    parser.add_argument(
        "--cmd-tau-topic",
        default=topics["cmd_tau"],
        help="Topic providing commanded efforts.",
    )
    parser.add_argument(
        "--ctrl-state-topic",
        default=topics["ctrl_state"],
        help="JointTrajectoryControllerState topic.",
    )
    parser.add_argument(
        "--use-header-time",
        action="store_true",
        default=True,
        help="Use ROS header time for PlotJuggler (default).",
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
    tabs = default_tabs(
        sim_tau_topic=args.sim_tau_topic,
        cmd_tau_topic=args.cmd_tau_topic,
        ctrl_state_topic=args.ctrl_state_topic,
        joint_count=args.joint_count,
    )
    out_path = write_layout(
        out_path=args.out,
        tabs=tabs,
        layout_name=args.layout_name,
        use_header_time=args.use_header_time,
    )
    print(f"Wrote PlotJuggler layout to: {out_path}")


if __name__ == "__main__":
    main()
