#!/bin/bash
# LidarRender 端到端冒烟：复用 zero-agent-sim 内的 roscore，启动 robokit:latest 的 lidar 模式。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CTR=zero-agent-sim
NAME=lidar-render-smoke
CONTAINER_SCRIPT=/workspace/ZeroAgent/runtime/RoboKit/tools/LidarRender/tests/publish_fake_odom.py
EXPECTED_GLOBAL_POINTS=40401
ODOM_PID=""
STARTED=0

cleanup() {
    if [ "$STARTED" -eq 1 ]; then
        bash "$ROOT/scripts/stop.sh" "$NAME" >/dev/null 2>&1 || true
    fi
    if [ -n "$ODOM_PID" ]; then
        docker exec "$CTR" kill -TERM "$ODOM_PID" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT

docker exec "$CTR" bash -lc 'test -f /opt/ros/noetic/setup.bash' || {
    echo "错误: 容器 $CTR 不存在或没有 ROS；先启动主链路或创建 zero-agent-sim" >&2
    exit 1
}
docker exec "$CTR" bash -lc 'source /opt/ros/noetic/setup.bash && rostopic list >/dev/null' || {
    echo "错误: $CTR 内没有可达的 ROS master；先启动主链路或手动 roscore" >&2
    exit 1
}

docker exec "$CTR" /usr/bin/python3 \
    /workspace/ZeroAgent/runtime/RoboKit/tools/LidarRender/tests/make_small_map.py

ODOM_PID=$(docker exec "$CTR" bash -lc "
    source /opt/ros/noetic/setup.bash
    /usr/bin/python3 $CONTAINER_SCRIPT --topic /super/sim/lidar_slam/odom \
        --x 0 --y 0 --z 0.5 >/tmp/lidar_fake_odom.log 2>&1 &
    echo \$!
")
[ -n "$ODOM_PID" ] || { echo "错误: fake odom 启动失败" >&2; exit 1; }

RUN_ARGS=(--detach --name "$NAME" "$ROOT/tests/data/small_map.pcd")
if [ "${LIDAR_USE_XVFB:-0}" = "1" ]; then
    RUN_ARGS=(--use-xvfb "${RUN_ARGS[@]}")
fi
bash "$ROOT/scripts/run.sh" "${RUN_ARGS[@]}"
STARTED=1

for _ in $(seq 1 40); do
    if docker exec "$CTR" bash -lc '
        source /opt/ros/noetic/setup.bash
        rostopic list | grep -qx /super/sim/cloud_registered
    ' >/dev/null 2>&1; then
        break
    fi
    sleep 0.5
done

LOCAL_WIDTH=$(docker exec "$CTR" bash -lc '
    source /opt/ros/noetic/setup.bash
    timeout 10 rostopic echo -n1 /super/sim/cloud_registered/width
' | head -1 | tr -d '\r' || true)
[ -n "$LOCAL_WIDTH" ] && [ "$LOCAL_WIDTH" -gt 0 ] || {
    echo "错误: 局部点云 width 无效: ${LOCAL_WIDTH:-<empty>}" >&2
    docker logs "$NAME" >&2 || true
    exit 1
}

GLOBAL_WIDTH=$(docker exec "$CTR" bash -lc '
    source /opt/ros/noetic/setup.bash
    timeout 20 rostopic echo -n1 /super/sim/global_pc/width
' | head -1 | tr -d '\r' || true)
[ "$GLOBAL_WIDTH" = "$EXPECTED_GLOBAL_POINTS" ] || {
    echo "错误: 全局点云 width=$GLOBAL_WIDTH，期望 $EXPECTED_GLOBAL_POINTS" >&2
    docker logs "$NAME" >&2 || true
    exit 1
}

if docker logs "$NAME" 2>&1 | grep -q 'Traceback'; then
    echo "错误: LidarRender 容器日志包含 Traceback" >&2
    docker logs "$NAME" >&2
    exit 1
fi

echo "SMOKE_OK local_width=$LOCAL_WIDTH global_width=$GLOBAL_WIDTH"
