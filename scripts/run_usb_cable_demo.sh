#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")/../../.."
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}" ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/usb_cable_demo_ros_logs}" MPLCONFIGDIR=/tmp/usb_cable_mpl
exec ros2 launch dual_fr3_moveit_config usb_cable.launch.py maniskill_python:="$PWD/.venv/bin/python" "$@"
