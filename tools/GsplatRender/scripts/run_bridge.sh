#!/bin/bash
set -eo pipefail
# ROS setup 脚本可能引用未定义变量，加载完成后再启用 -u。
source /opt/ros/noetic/setup.bash
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 -u "$ROOT/renderer/gsplat_ros_bridge.py" "$@"
