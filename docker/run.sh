#!/bin/bash
# 运行统一 RoboKit 容器：同一容器内启动 GsplatRender 与 LidarRender。
#
# 用法:
#   bash docker/run.sh [选项]
#
# 选项:
#   --name NAME          容器名，默认 robokit
#   --ply FILE           PLY 场景（GsplatRender）；只给 --pcd 时可不传
#   --pcd FILE           PCD 场景（LidarRender）；只给 --ply 时可不传
#   --odom-topic T       两个 bridge 订阅的 odom 话题，默认 /super/sim/lidar_slam/odom
#   --log-dir DIR        宿主日志目录，挂载到容器 /logs；两个工具分别写日志
#   -d, --detach         后台运行；默认前台运行且退出后删除容器
#   -h, --help
#
# 环境变量:
#   IMAGE          镜像名，默认 robokit:latest
#   ROS_MASTER_URI 设置时透传给容器
#
# 至少给出 --ply 或 --pcd；只给一个时只启动对应工具。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-robokit:latest}"
NAME="${NAME:-robokit}"
ODOM_TOPIC="/super/sim/lidar_slam/odom"
PLY_FILE=""
PCD_FILE=""
LOG_DIR=""
DETACH=0

usage() {
    cat >&2 <<'EOF'
用法: bash docker/run.sh [选项]

选项:
  --name NAME          容器名，默认 robokit
  --ply FILE           PLY 场景（GsplatRender）
  --pcd FILE           PCD 场景（LidarRender）
  --odom-topic T       两个 bridge 订阅的 odom 话题，默认 /super/sim/lidar_slam/odom
  --log-dir DIR        宿主日志目录，挂载到容器 /logs
  -d, --detach         后台运行；默认前台运行且退出后删除容器
  -h, --help

至少给出 --ply 或 --pcd；只给一个时只启动对应工具。
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --name)
            [ "$#" -ge 2 ] || { echo "错误: --name 需要参数" >&2; exit 2; }
            NAME="$2"
            shift 2
            ;;
        --ply)
            [ "$#" -ge 2 ] || { echo "错误: --ply 需要参数" >&2; exit 2; }
            PLY_FILE="$2"
            shift 2
            ;;
        --pcd)
            [ "$#" -ge 2 ] || { echo "错误: --pcd 需要参数" >&2; exit 2; }
            PCD_FILE="$2"
            shift 2
            ;;
        --odom-topic)
            [ "$#" -ge 2 ] || { echo "错误: --odom-topic 需要参数" >&2; exit 2; }
            ODOM_TOPIC="$2"
            shift 2
            ;;
        --log-dir)
            [ "$#" -ge 2 ] || { echo "错误: --log-dir 需要参数" >&2; exit 2; }
            LOG_DIR="$2"
            shift 2
            ;;
        -d|--detach)
            DETACH=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "错误: 未知选项 $1" >&2
            usage
            exit 2
            ;;
    esac
done

[ -n "$PLY_FILE" ] || [ -n "$PCD_FILE" ] || { usage; exit 2; }

PLY_ABS=""
if [ -n "$PLY_FILE" ]; then
    [ -f "$PLY_FILE" ] || { echo "错误: PLY 文件不存在: $PLY_FILE" >&2; exit 1; }
    PLY_ABS="$(realpath "$PLY_FILE")"
fi
PCD_ABS=""
if [ -n "$PCD_FILE" ]; then
    [ -f "$PCD_FILE" ] || { echo "错误: PCD 文件不存在: $PCD_FILE" >&2; exit 1; }
    PCD_ABS="$(realpath "$PCD_FILE")"
fi

command -v docker >/dev/null 2>&1 || { echo "错误: 没有可用的 docker" >&2; exit 1; }
docker image inspect "$IMAGE" >/dev/null 2>&1 || {
    echo "错误: 镜像不存在: $IMAGE" >&2
    echo "先运行: bash $ROOT/docker/build.sh" >&2
    exit 1
}
if docker container inspect "$NAME" >/dev/null 2>&1; then
    echo "错误: 容器名已存在: $NAME" >&2
    echo "先运行: docker rm -f $NAME" >&2
    exit 1
fi
command -v nvidia-smi >/dev/null 2>&1 || echo "提示: 未找到 nvidia-smi，容器可能无法使用 GPU" >&2

DOCKER_ARGS=(run --gpus all --net=host)
if [ "$DETACH" -eq 1 ]; then
    DOCKER_ARGS+=(-d --name "$NAME")
else
    DOCKER_ARGS+=(--rm --name "$NAME")
fi

DOCKER_ARGS+=(
    -e "GSPLAT_ODOM_TOPIC=$ODOM_TOPIC"
    -e "LIDAR_ODOM_TOPIC=$ODOM_TOPIC"
    -e "QT_X11_NO_MITSHM=1"
    -e "NVIDIA_DRIVER_CAPABILITIES=all"
)
if [ -n "${DISPLAY:-}" ]; then
    DOCKER_ARGS+=(-e "DISPLAY=$DISPLAY" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw")
fi
if [ -n "${ROS_MASTER_URI:-}" ]; then
    DOCKER_ARGS+=(-e "ROS_MASTER_URI=$ROS_MASTER_URI")
fi
if [ -n "$LOG_DIR" ]; then
    mkdir -p "$LOG_DIR"
    DOCKER_ARGS+=(-e "ROBOKIT_LOG_DIR=/logs" -v "$(realpath "$LOG_DIR"):/logs:rw")
fi

if [ -n "$PLY_ABS" ]; then
    DOCKER_ARGS+=(
        -v "$PLY_ABS:/data/scene.ply:ro"
        -v "$ROOT/tools/GsplatRender/runtime:/opt/gsplat-render/runtime:ro"
        -v "$ROOT/tools/GsplatRender/config:/opt/gsplat-render/config:ro"
        -v "$ROOT/tools/GsplatRender/docker/entrypoint.sh:/opt/gsplat-render/docker/entrypoint.sh:ro"
    )
fi
if [ -n "$PCD_ABS" ]; then
    DOCKER_ARGS+=(
        -v "$PCD_ABS:/data/scene.pcd:ro"
        -v "$ROOT/tools/LidarRender/runtime:/opt/lidar-render/runtime:ro"
        -v "$ROOT/tools/LidarRender/config:/opt/lidar-render/config:ro"
        -v "$ROOT/tools/LidarRender/docker/entrypoint.sh:/opt/lidar-render/docker/entrypoint.sh:ro"
    )
    if [ -n "${DISPLAY:-}" ]; then
        command -v xhost >/dev/null 2>&1 && xhost +local: >/dev/null 2>&1 || true
    fi
fi

if [ -n "$PLY_ABS" ] && [ -n "$PCD_ABS" ]; then
    DOCKER_ARGS+=("$IMAGE" both /data/scene.ply /data/scene.pcd)
elif [ -n "$PLY_ABS" ]; then
    DOCKER_ARGS+=("$IMAGE" gsplat /data/scene.ply)
else
    DOCKER_ARGS+=("$IMAGE" lidar /data/scene.pcd)
fi

docker "${DOCKER_ARGS[@]}"

if [ "$DETACH" -eq 1 ]; then
    echo "容器已在后台启动: $NAME"
    echo "查看日志: docker logs -f $NAME"
    echo "停止: docker rm -f $NAME"
fi
