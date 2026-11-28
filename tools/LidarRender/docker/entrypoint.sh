#!/bin/bash
# LidarRender 容器入口：同一容器内启动 C++ TCP renderer 和 ROS 1 bridge。
set -euo pipefail

if [ "$#" -lt 1 ]; then
    echo "用法: entrypoint.sh <pcd_file|ply_file>" >&2
    exit 2
fi
PCD_FILE="$1"

LIDAR_HOST="${LIDAR_HOST:-127.0.0.1}"
LIDAR_POSE_PORT="${LIDAR_POSE_PORT:-19082}"
LIDAR_CLOUD_PORT="${LIDAR_CLOUD_PORT:-19083}"
LIDAR_ODOM_TOPIC="${LIDAR_ODOM_TOPIC:-/super/sim/lidar_slam/odom}"
LIDAR_LOCAL_TOPIC="${LIDAR_LOCAL_TOPIC:-/super/sim/cloud_registered}"
LIDAR_GLOBAL_TOPIC="${LIDAR_GLOBAL_TOPIC:-/super/sim/global_pc}"
LIDAR_FRAME_ID="${LIDAR_FRAME_ID:-world}"
LIDAR_CONFIG="${LIDAR_CONFIG:-/opt/lidar-render/config/params.yaml}"
LIDAR_USE_XVFB="${LIDAR_USE_XVFB:-0}"

[ -f "$PCD_FILE" ] || { echo "错误: 场景文件不存在: $PCD_FILE" >&2; exit 2; }

# PLY 点云先转成 binary PCD；renderer 和 bridge 都只读 PCD。
if [ "$(head -c 3 "$PCD_FILE" 2>/dev/null)" = "ply" ]; then
    CONVERTED_PCD="/tmp/lidar_render_scene.pcd"
    echo "[LidarRender] 转换 PLY -> PCD: $PCD_FILE"
    /usr/bin/python3 -u /opt/lidar-render/runtime/ply_to_pcd.py \
        "$PCD_FILE" "$CONVERTED_PCD"
    PCD_FILE="$CONVERTED_PCD"
fi

RENDER_CMD=()
if [ "$LIDAR_USE_XVFB" = "1" ]; then
    RENDER_CMD=(xvfb-run -a)
elif [ -z "${DISPLAY:-}" ]; then
    echo "错误: 未设置 DISPLAY；LidarRender 需要 X11（或设置 LIDAR_USE_XVFB=1）" >&2
    exit 2
fi

"${RENDER_CMD[@]}" /opt/lidar-render/bin/lidar_render_tcp \
    --pcd_file "$PCD_FILE" \
    --config_file "$LIDAR_CONFIG" \
    --host "$LIDAR_HOST" \
    --pose_port "$LIDAR_POSE_PORT" \
    --cloud_port "$LIDAR_CLOUD_PORT" \
    --frame_id "$LIDAR_FRAME_ID" &
renderer_pid=$!

set +u
# shellcheck disable=SC1090
source /opt/ros/noetic/setup.bash
set -u

/usr/bin/python3 -u /opt/lidar-render/runtime/lidar_ros_bridge.py \
    --host "$LIDAR_HOST" \
    --pose-port "$LIDAR_POSE_PORT" \
    --cloud-port "$LIDAR_CLOUD_PORT" \
    --odom-topic "$LIDAR_ODOM_TOPIC" \
    --local-cloud-topic "$LIDAR_LOCAL_TOPIC" \
    --global-cloud-topic "$LIDAR_GLOBAL_TOPIC" \
    --pcd-file "$PCD_FILE" \
    --frame-id "$LIDAR_FRAME_ID" &
bridge_pid=$!

cleanup() {
    kill -TERM "$bridge_pid" "$renderer_pid" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT TERM INT

status=0
wait -n || status=$?
exit "$status"
