#!/usr/bin/env bash
set -euo pipefail

PKG=rlc_robot_descriptions
XACRO_REL=robots/kinova_gen3/urdf/gen3_7dof_vision_gz.urdf.xacro

SHARE="$(ros2 pkg prefix --share "$PKG")"
XACRO="$SHARE/$XACRO_REL"

CONTROLLERS_YAML_DEFAULT="$SHARE/config/gen3_gz_controllers.yaml"
CONTROLLERS_YAML="${1:-$CONTROLLERS_YAML_DEFAULT}"

ros2 run xacro xacro "$XACRO" "controllers_yaml:=$CONTROLLERS_YAML"
