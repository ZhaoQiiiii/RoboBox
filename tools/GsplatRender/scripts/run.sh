#!/bin/bash
# 运行 GsplatRender 容器：同一容器内启动 TCP renderer + ROS 1 bridge。
#
# 用法:
#   bash scripts/run.sh [选项] <ply_file> [renderer 额外参数...]
#
# 选项:
#   --name NAME        容器名，默认 gsplat-render
#   --params FILE      渲染参数 yaml（宿主路径），覆盖镜像内 config/params.yaml
#   --host ADDR        renderer 绑定地址，也是 bridge 连接地址，默认 127.0.0.1
#   --pose-port PORT   默认 19080
#   --frame-port PORT  默认 19081
#   --odom-topic T     bridge 订阅的位姿话题，默认 /super/sim/lidar_slam/odom
#   -d, --detach       后台运行；默认前台运行且退出后删除容器
#   -h, --help
#
# 镜像只提供依赖；runtime/、config/ 与 docker/entrypoint.sh 始终挂载宿主目录，
# 改完重跑本脚本或 docker restart gsplat-render 即生效，不需要重建镜像。
#
# 环境变量:
#   IMAGE          镜像名，默认 gsplat-render:noetic
#   ROS_MASTER_URI 设置时透传给容器；否则用容器默认 http://localhost:11311
#
# 示例:
#   bash scripts/run.sh /path/scene.ply
#   bash scripts/run.sh --detach --params /path/params.yaml /path/scene.ply
#   bash scripts/run.sh --odom-topic /odom /path/scene.ply
#
# 云台模式（仅使用 yaw）由容器 entrypoint 默认开启，无需额外传参。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-gsplat-render:noetic}"
NAME="${NAME:-gsplat-render}"
HOST="127.0.0.1"
POSE_PORT="19080"
FRAME_PORT="19081"
ODOM_TOPIC="/super/sim/lidar_slam/odom"
PLY_FILE=""
PARAMS_FILE=""
DETACH=0
EXTRA=()

usage() {
    cat >&2 <<'EOF'
用法: bash scripts/run.sh [选项] <ply_file> [renderer 额外参数...]

选项:
  --name NAME        容器名，默认 gsplat-render
  --params FILE      渲染参数 yaml（宿主路径），覆盖镜像内 config/params.yaml
  --host ADDR        renderer 绑定地址，也是 bridge 连接地址，默认 127.0.0.1
  --pose-port PORT   默认 19080
  --frame-port PORT  默认 19081
  --odom-topic T     bridge 订阅的位姿话题，默认 /super/sim/lidar_slam/odom
  -d, --detach       后台运行；默认前台运行且退出后删除容器
  -h, --help

镜像只提供依赖；runtime/、config/ 与 docker/entrypoint.sh 始终挂载宿主目录，
改完重跑本脚本或 docker restart gsplat-render 即生效。

云台模式默认开启，无需传 --enable_gimbal。
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
        --frame-port)
            [ "$#" -ge 2 ] || { echo "错误: --frame-port 需要参数" >&2; exit 2; }
            FRAME_PORT="$2"
            shift 2
            ;;
        --odom-topic)
            [ "$#" -ge 2 ] || { echo "错误: --odom-topic 需要参数" >&2; exit 2; }
            ODOM_TOPIC="$2"
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
        --)
            shift
            PLY_FILE="${1:-}"
            [ "$#" -gt 0 ] && shift
            EXTRA=("$@")
            break
            ;;
        -*)
            echo "错误: 未知选项 $1（renderer 参数要放在 PLY 之后）" >&2
            usage
            exit 2
            ;;
        *)
            PLY_FILE="$1"
            shift
            EXTRA=("$@")
            break
            ;;
    esac
done

if [ -z "$PLY_FILE" ]; then
    usage
    exit 2
fi
[ -f "$PLY_FILE" ] || { echo "错误: PLY 文件不存在: $PLY_FILE" >&2; exit 1; }
PLY_ABS="$(realpath "$PLY_FILE")"

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
[ -x "$ROOT/docker/entrypoint.sh" ] || {
    echo "错误: $ROOT/docker/entrypoint.sh 不存在或不可执行" >&2
    exit 1
}
if docker container inspect "$NAME" >/dev/null 2>&1; then
    echo "错误: 容器名已存在: $NAME" >&2
    echo "先运行: bash $ROOT/scripts/stop.sh $NAME" >&2
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
    -e "GSPLAT_HOST=$HOST"
    -e "GSPLAT_POSE_PORT=$POSE_PORT"
    -e "GSPLAT_FRAME_PORT=$FRAME_PORT"
    -e "GSPLAT_ODOM_TOPIC=$ODOM_TOPIC"
)
if [ -n "${ROS_MASTER_URI:-}" ]; then
    DOCKER_ARGS+=(-e "ROS_MASTER_URI=$ROS_MASTER_URI")
fi

DOCKER_ARGS+=(-v "$PLY_ABS:/data/scene.ply:ro")
if [ -n "$PARAMS_ABS" ]; then
    DOCKER_ARGS+=(-v "$PARAMS_ABS:/opt/gsplat-render/config/params.yaml:ro")
fi
# 代码、默认配置和 entrypoint 始终用宿主当前版本，镜像只提供依赖。
DOCKER_ARGS+=(
    -v "$ROOT/runtime:/opt/gsplat-render/runtime:ro"
    -v "$ROOT/config:/opt/gsplat-render/config:ro"
    -v "$ROOT/docker/entrypoint.sh:/opt/gsplat-render/docker/entrypoint.sh:ro"
)

DOCKER_ARGS+=("$IMAGE" /data/scene.ply)
if [ "${#EXTRA[@]}" -gt 0 ]; then
    DOCKER_ARGS+=("${EXTRA[@]}")
fi

docker "${DOCKER_ARGS[@]}"

if [ "$DETACH" -eq 1 ]; then
    echo "容器已在后台启动: $NAME"
    echo "查看日志: docker logs -f $NAME"
    echo "停止: bash $ROOT/scripts/stop.sh $NAME"
    echo "修改 runtime/、config/ 或 docker/entrypoint.sh 后执行 docker restart $NAME 生效，无需重建镜像"
fi
