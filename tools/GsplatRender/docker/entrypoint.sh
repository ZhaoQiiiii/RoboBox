#!/bin/bash
# GsplatRender 容器入口：同一容器内启动 TCP renderer 和 ROS 1 bridge。
# renderer 用 venv 的 Python 3.10；bridge 用 ROS Noetic 的系统 Python 3.8。
#
# 第一个参数是 PLY 路径，其余参数原样传给 renderer。云台模式默认开启。
# 环境变量：
#   GSPLAT_HOST          默认 127.0.0.1（renderer 绑定地址，也是 bridge 连接地址）
#   GSPLAT_POSE_PORT     默认 19080
#   GSPLAT_FRAME_PORT    默认 19081
#   GSPLAT_ODOM_TOPIC    默认 /super/sim/lidar_slam/odom
#
# bridge 需要可达的 ROS master；镜像不负责启动 roscore。
set -euo pipefail

APP_DIR=/opt/gsplat-render
RENDERER_PY=/opt/gsplat-venv/bin/python
BRIDGE_PY=/usr/bin/python3
ROS_SETUP=/opt/ros/noetic/setup.bash

POSE_PORT="${GSPLAT_POSE_PORT:-19080}"
FRAME_PORT="${GSPLAT_FRAME_PORT:-19081}"
HOST="${GSPLAT_HOST:-127.0.0.1}"
ODOM_TOPIC="${GSPLAT_ODOM_TOPIC:-/super/sim/lidar_slam/odom}"
PARAMS_FILE="$APP_DIR/config/params.yaml"

PLY_FILE="${1:-${PLY_FILE:-}}"
if [ "$#" -gt 0 ]; then
    shift
fi

if [ -z "$PLY_FILE" ]; then
    echo "错误: 缺少 PLY 文件；用法: docker run ... <ply_file>" >&2
    exit 2
fi
if [ ! -f "$PLY_FILE" ]; then
    echo "错误: PLY 文件不存在: $PLY_FILE" >&2
    echo "挂载示例: -v /host/scene.ply:/data/scene.ply:ro，然后传 /data/scene.ply" >&2
    exit 2
fi

run_renderer() {
    exec "$RENDERER_PY" -u "$APP_DIR/runtime/gsplat_render_tcp.py" \
        --ply_file "$PLY_FILE" \
        --params_file "$PARAMS_FILE" \
        --host "$HOST" \
        --pose-port "$POSE_PORT" \
        --frame-port "$FRAME_PORT" \
        --enable_gimbal "$@"
}

run_bridge() {
    set +u
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    set -u
    exec "$BRIDGE_PY" -u "$APP_DIR/runtime/gsplat_ros_bridge.py" \
        --host "$HOST" \
        --pose-port "$POSE_PORT" \
        --frame-port "$FRAME_PORT" \
        --odom-topic "$ODOM_TOPIC"
}

renderer_pid=""
bridge_pid=""

cleanup() {
    local status=$?
    trap - EXIT TERM INT
    for pid in "$bridge_pid" "$renderer_pid"; do
        if [ -n "$pid" ]; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
    exit "$status"
}
trap cleanup EXIT TERM INT

run_renderer "$@" &
renderer_pid=$!

run_bridge &
bridge_pid=$!

wait -n
