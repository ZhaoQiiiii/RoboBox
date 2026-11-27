#!/bin/bash
# 运行 LidarRender 容器：同一容器内启动 TCP renderer + ROS 1 bridge。
#
# 用法:
#   bash scripts/run.sh [选项] <pcd_file>
#
# 环境变量:
#   IMAGE          镜像名，默认 robokit:latest
#   ROS_MASTER_URI 设置时透传给容器
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-robokit:latest}"
NAME="${NAME:-lidar-render}"
HOST="127.0.0.1"
POSE_PORT="19082"
CLOUD_PORT="19083"
ODOM_TOPIC="/super/sim/lidar_slam/odom"
LOCAL_TOPIC="/super/sim/cloud_registered"
GLOBAL_TOPIC="/super/sim/global_pc"
FRAME_ID="world"
PARAMS_FILE=""
PCD_FILE=""
DETACH=0
USE_XVFB=0

usage() {
    cat >&2 <<'EOF'
用法: bash scripts/run.sh [选项] <pcd_file>

选项:
  --name NAME               容器名，默认 lidar-render
  --params FILE             参数 YAML（宿主路径），覆盖镜像内 config/params.yaml
  --host ADDR               renderer 绑定地址，也是 bridge 连接地址，默认 127.0.0.1
  --pose-port PORT          默认 19082
  --cloud-port PORT         默认 19083
  --odom-topic T            bridge 订阅的 odom 话题，默认 /super/sim/lidar_slam/odom
  --local-cloud-topic T     局部点云话题，默认 /super/sim/cloud_registered
  --global-cloud-topic T    全局点云话题，默认 /super/sim/global_pc
  --frame-id FRAME          点云 frame_id，默认 world
  --use-xvfb                在容器内用 xvfb-run 软件渲染（无宿主 X11 时）
  -d, --detach              后台运行；默认前台运行且退出后删除容器
  -h, --help

示例:
  bash scripts/run.sh /path/scene.pcd
  bash scripts/run.sh --detach --params /path/params.yaml /path/scene.pcd
  bash scripts/run.sh --use-xvfb /path/scene.pcd
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --name)
            [ "$#" -ge 2 ] || { echo "错误: --name 需要参数" >&2; exit 2; }
            NAME="$2"
            shift 2
            ;;
        --params)
            [ "$#" -ge 2 ] || { echo "错误: --params 需要参数" >&2; exit 2; }
            PARAMS_FILE="$2"
            shift 2
            ;;
        --host)
            [ "$#" -ge 2 ] || { echo "错误: --host 需要参数" >&2; exit 2; }
            HOST="$2"
            shift 2
            ;;
        --pose-port)
            [ "$#" -ge 2 ] || { echo "错误: --pose-port 需要参数" >&2; exit 2; }
            POSE_PORT="$2"
            shift 2
            ;;
        --cloud-port)
            [ "$#" -ge 2 ] || { echo "错误: --cloud-port 需要参数" >&2; exit 2; }
            CLOUD_PORT="$2"
            shift 2
            ;;
        --odom-topic)
            [ "$#" -ge 2 ] || { echo "错误: --odom-topic 需要参数" >&2; exit 2; }
            ODOM_TOPIC="$2"
            shift 2
            ;;
        --local-cloud-topic)
            [ "$#" -ge 2 ] || { echo "错误: --local-cloud-topic 需要参数" >&2; exit 2; }
            LOCAL_TOPIC="$2"
            shift 2
            ;;
        --global-cloud-topic)
            [ "$#" -ge 2 ] || { echo "错误: --global-cloud-topic 需要参数" >&2; exit 2; }
            GLOBAL_TOPIC="$2"
            shift 2
            ;;
        --frame-id)
            [ "$#" -ge 2 ] || { echo "错误: --frame-id 需要参数" >&2; exit 2; }
            FRAME_ID="$2"
            shift 2
            ;;
        --use-xvfb)
            USE_XVFB=1
            shift
            ;;
        -d|--detach)
            DETACH=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            echo "错误: 未知选项 $1" >&2
            usage
            exit 2
            ;;
        *)
            PCD_FILE="$1"
            shift
            break
            ;;
    esac
done

if [ -z "$PCD_FILE" ]; then
    usage
    exit 2
fi
[ -f "$PCD_FILE" ] || { echo "错误: PCD 文件不存在: $PCD_FILE" >&2; exit 1; }
PCD_ABS="$(realpath "$PCD_FILE")"

PARAMS_ABS=""
if [ -n "$PARAMS_FILE" ]; then
    [ -f "$PARAMS_FILE" ] || { echo "错误: 参数文件不存在: $PARAMS_FILE" >&2; exit 1; }
    PARAMS_ABS="$(realpath "$PARAMS_FILE")"
fi

command -v docker >/dev/null 2>&1 || { echo "错误: 没有可用的 docker" >&2; exit 1; }
docker image inspect "$IMAGE" >/dev/null 2>&1 || {
    echo "错误: 镜像不存在: $IMAGE" >&2
    echo "先运行: bash $ROOT/scripts/build.sh" >&2
    exit 1
}
if docker container inspect "$NAME" >/dev/null 2>&1; then
    echo "错误: 容器名已存在: $NAME" >&2
    echo "先运行: bash $ROOT/scripts/stop.sh $NAME" >&2
    exit 1
fi
command -v nvidia-smi >/dev/null 2>&1 || echo "提示: 未找到 nvidia-smi，容器可能无法使用 GPU" >&2
if command -v xhost >/dev/null 2>&1; then
    xhost +local: >/dev/null 2>&1 || true
fi

DOCKER_ARGS=(run --gpus all --net=host)
if [ "$DETACH" -eq 1 ]; then
    DOCKER_ARGS+=(-d --name "$NAME")
else
    DOCKER_ARGS+=(--rm --name "$NAME")
fi

DOCKER_ARGS+=(
    -e "DISPLAY=${DISPLAY:-}"
    -e "QT_X11_NO_MITSHM=1"
    -e "NVIDIA_DRIVER_CAPABILITIES=all"
    -e "LIDAR_HOST=$HOST"
    -e "LIDAR_POSE_PORT=$POSE_PORT"
    -e "LIDAR_CLOUD_PORT=$CLOUD_PORT"
    -e "LIDAR_ODOM_TOPIC=$ODOM_TOPIC"
    -e "LIDAR_LOCAL_TOPIC=$LOCAL_TOPIC"
    -e "LIDAR_GLOBAL_TOPIC=$GLOBAL_TOPIC"
    -e "LIDAR_FRAME_ID=$FRAME_ID"
    -e "LIDAR_USE_XVFB=$USE_XVFB"
)
if [ -n "${ROS_MASTER_URI:-}" ]; then
    DOCKER_ARGS+=(-e "ROS_MASTER_URI=$ROS_MASTER_URI")
fi

DOCKER_ARGS+=(
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw"
    -v "$PCD_ABS:/data/scene.pcd:ro"
    -v "$ROOT/runtime:/opt/lidar-render/runtime:ro"
    -v "$ROOT/config:/opt/lidar-render/config:ro"
    -v "$ROOT/docker/entrypoint.sh:/opt/lidar-render/docker/entrypoint.sh:ro"
)
if [ -n "$PARAMS_ABS" ]; then
    DOCKER_ARGS+=(-v "$PARAMS_ABS:/opt/lidar-render/config/params.yaml:ro")
fi

if [ "$USE_XVFB" -eq 1 ]; then
    echo "提示: 使用 xvfb-run 软件渲染，性能会明显低于宿主 X11" >&2
fi

DOCKER_ARGS+=("$IMAGE" lidar /data/scene.pcd)
docker "${DOCKER_ARGS[@]}"

if [ "$DETACH" -eq 1 ]; then
    echo "容器已在后台启动: $NAME"
    echo "查看日志: docker logs -f $NAME"
    echo "停止: bash $ROOT/scripts/stop.sh $NAME"
fi
