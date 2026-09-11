#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")/../../.."
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/usb_cable_demo_ros_logs MPLCONFIGDIR=/tmp/usb_cable_mpl
exec /usr/bin/python3 src/dual_fr3_moveit_config/test/run_usb_cable_ros.py "$@"
