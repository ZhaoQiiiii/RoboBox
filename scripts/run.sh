#!/bin/bash
# 运行统一 RoboKit 容器：同一容器内启动 GsplatRender 与 LidarRender。
#
# 用法:
#   bash scripts/run.sh [选项]
#
# 选项:
#   --name NAME          容器名，默认 robokit
#   --ply FILE           PLY 场景（GsplatRender）；只给 --pcd 时可不传
#   --pcd FILE           PCD 或点云 PLY 场景（LidarRender）；只给 --ply 时可不传
#   --odom-topic T       两个 bridge 订阅的 odom 话题，默认 /super/sim/lidar_slam/odom
#   --lidar-odom-topic T 单独指定 LidarRender 的 odom 话题
#   --lidar-local-topic T / --lidar-global-topic T   LidarRender 点云输出话题
#   --log-dir DIR        宿主日志目录，挂载到容器 /logs；两个工具分别写日志
#   --rerun              打开 Rerun 可视化（默认关闭）
#   --rerun-url URL      Rerun gRPC server，默认 rerun+http://127.0.0.1:9877/proxy
#   --rerun-serve        让 bridge 自己起 gRPC server（standalone 用）
#   --rerun-port N       自起 server 的端口，默认 9880
#   --rerun-image-topic T / --rerun-depth-topic T
#                        要转发的 ROS 图像话题；depth 默认关闭
#   --rerun-cloud-topic T[@BASE]
#                        点云话题，可重复；@BASE 指定 Rerun 实体路径
#   --rerun-path-topic T[@BASE]
#                        nav_msgs/Path 话题，可重复；@BASE 指定 Rerun 实体路径
#   --rerun-cloud-hz N   点云上限频率，默认 5
#   --rerun-odom-topic T 转发 odom 轨迹，默认关闭
#   --rerun-odom-hz N    odom 上限频率，默认 10
#   --rerun-marker-topic T[@BASE]
#                        visualization_msgs/Marker 话题，可重复
#   --rerun-marker-array-topic T[@BASE]
#                        visualization_msgs/MarkerArray 话题，可重复
#   --rerun-marker-hz N  Marker 上限频率，默认 5
#   --rerun-max-points N 点云抽稀上限，默认 20000
#   --rerun-app NAME     Rerun application id，默认 ZeroAgent
#   --rerun-recording-id ID
#                        与 DimOS viewer 共享的 recording id，默认不指定
#   --rerun-entity-prefix PREFIX
#                        Rerun 实体前缀，默认 /world/robokit
#   --rerun-robot-body   从 odom 绘制代理机体
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
LIDAR_ODOM_TOPIC=""
LIDAR_LOCAL_TOPIC="/super/sim/cloud_registered"
LIDAR_GLOBAL_TOPIC="/super/sim/global_pc"
PLY_FILE=""
PCD_FILE=""
LOG_DIR=""
DETACH=0
RERUN=0
RERUN_URL="rerun+http://127.0.0.1:9877/proxy"
RERUN_SERVE=0
RERUN_PORT=9880
RERUN_IMAGE_TOPIC="/gsplat_render/camera0/color/image/compressed"
RERUN_DEPTH_TOPIC=""
RERUN_CLOUD_TOPICS=""
RERUN_PATH_TOPICS=""
RERUN_CLOUD_HZ=5
RERUN_ODOM_TOPIC=""
RERUN_ODOM_HZ=10
RERUN_MARKER_TOPICS=""
RERUN_MARKER_ARRAY_TOPICS=""
RERUN_MARKER_HZ=5
RERUN_MAX_POINTS=20000
RERUN_APP=ZeroAgent
RERUN_RECORDING_ID=""
RERUN_ENTITY_PREFIX="/world/robokit"
RERUN_ROBOT_BODY=0

usage() {
    cat >&2 <<'EOF'
用法: bash scripts/run.sh [选项]

选项:
  --name NAME          容器名，默认 robokit
  --ply FILE           PLY 场景（GsplatRender）
  --pcd FILE           PCD 或点云 PLY 场景（LidarRender）
  --odom-topic T       两个 bridge 订阅的 odom 话题，默认 /super/sim/lidar_slam/odom
  --lidar-odom-topic T 单独指定 LidarRender 的 odom 话题
  --lidar-local-topic T / --lidar-global-topic T   LidarRender 点云输出话题
  --log-dir DIR        宿主日志目录，挂载到容器 /logs
  --rerun              打开 Rerun 可视化（默认关闭）
  --rerun-url URL      Rerun gRPC server，默认 rerun+http://127.0.0.1:9877/proxy
  --rerun-serve        让 bridge 自己起 gRPC server（standalone 用）
  --rerun-port N       自起 server 的端口，默认 9880
  --rerun-image-topic T / --rerun-depth-topic T
                       要转发的 ROS 话题；depth 默认关闭
  --rerun-cloud-topic T[@BASE]
                       点云话题，可重复；@BASE 指定 Rerun 实体路径
  --rerun-path-topic T[@BASE]
                       nav_msgs/Path 话题，可重复；@BASE 指定 Rerun 实体路径
  --rerun-cloud-hz N   点云上限频率，默认 5
  --rerun-odom-topic T 转发 odom 轨迹，默认关闭
  --rerun-odom-hz N    odom 上限频率，默认 10
  --rerun-marker-topic T
                       visualization_msgs/Marker 话题，可重复
  --rerun-marker-array-topic T
                       visualization_msgs/MarkerArray 话题，可重复
  --rerun-marker-hz N  Marker 上限频率，默认 5
  --rerun-max-points N 点云抽稀上限，默认 20000
  --rerun-app NAME     Rerun application id，默认 ZeroAgent
  --rerun-recording-id ID
                       与 DimOS viewer 共享的 recording id，默认不指定
  --rerun-entity-prefix PREFIX
                       Rerun 实体前缀，默认 /world/robokit
  --rerun-robot-body   从 odom 绘制代理机体
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
        --lidar-odom-topic|--lidar-local-topic|--lidar-global-topic)
            [ -n "${2:-}" ] || { echo "错误: $1 需要话题名" >&2; exit 2; }
            case "$1" in
                --lidar-odom-topic) LIDAR_ODOM_TOPIC="$2" ;;
                --lidar-local-topic) LIDAR_LOCAL_TOPIC="$2" ;;
                --lidar-global-topic) LIDAR_GLOBAL_TOPIC="$2" ;;
            esac
            shift 2
            ;;
        --log-dir)
            [ "$#" -ge 2 ] || { echo "错误: --log-dir 需要参数" >&2; exit 2; }
            LOG_DIR="$2"
            shift 2
            ;;
        --rerun)
            RERUN=1
            shift
            ;;
        --rerun-url)
            [ "$#" -ge 2 ] || { echo "错误: --rerun-url 需要参数" >&2; exit 2; }
            RERUN=1
            RERUN_URL="$2"
            shift 2
            ;;
        --rerun-serve)
            RERUN=1
            RERUN_SERVE=1
            shift
            ;;
        --rerun-port)
            [ "$#" -ge 2 ] || { echo "错误: --rerun-port 需要参数" >&2; exit 2; }
            RERUN=1
            RERUN_PORT="$2"
            shift 2
            ;;
        --rerun-image-topic|--rerun-depth-topic)
            [ "$#" -ge 2 ] || { echo "错误: $1 需要参数" >&2; exit 2; }
            RERUN=1
            case "$1" in
                --rerun-image-topic) RERUN_IMAGE_TOPIC="$2" ;;
                --rerun-depth-topic) RERUN_DEPTH_TOPIC="$2" ;;
            esac
            shift 2
            ;;
        --rerun-cloud-topic)
            [ "$#" -ge 2 ] || { echo "错误: --rerun-cloud-topic 需要参数" >&2; exit 2; }
            RERUN=1
            RERUN_CLOUD_TOPICS="$RERUN_CLOUD_TOPICS $2"
            shift 2
            ;;
        --rerun-path-topic)
            [ "$#" -ge 2 ] || { echo "错误: --rerun-path-topic 需要参数" >&2; exit 2; }
            RERUN=1
            RERUN_PATH_TOPICS="$RERUN_PATH_TOPICS $2"
            shift 2
            ;;
        --rerun-cloud-hz)
            [ "$#" -ge 2 ] || { echo "错误: --rerun-cloud-hz 需要参数" >&2; exit 2; }
            RERUN=1
            RERUN_CLOUD_HZ="$2"
            shift 2
            ;;
        --rerun-odom-topic)
            [ "$#" -ge 2 ] || { echo "错误: --rerun-odom-topic 需要参数" >&2; exit 2; }
            RERUN=1
            RERUN_ODOM_TOPIC="$2"
            shift 2
            ;;
        --rerun-odom-hz)
            [ "$#" -ge 2 ] || { echo "错误: --rerun-odom-hz 需要参数" >&2; exit 2; }
            RERUN=1
            RERUN_ODOM_HZ="$2"
            shift 2
            ;;
        --rerun-marker-topic)
            [ "$#" -ge 2 ] || { echo "错误: --rerun-marker-topic 需要参数" >&2; exit 2; }
            RERUN=1
            RERUN_MARKER_TOPICS="$RERUN_MARKER_TOPICS $2"
            shift 2
            ;;
        --rerun-marker-array-topic)
            [ "$#" -ge 2 ] || { echo "错误: --rerun-marker-array-topic 需要参数" >&2; exit 2; }
            RERUN=1
            RERUN_MARKER_ARRAY_TOPICS="$RERUN_MARKER_ARRAY_TOPICS $2"
            shift 2
            ;;
        --rerun-marker-hz)
            [ "$#" -ge 2 ] || { echo "错误: --rerun-marker-hz 需要参数" >&2; exit 2; }
            RERUN=1
            RERUN_MARKER_HZ="$2"
            shift 2
            ;;
        --rerun-max-points)
            [ "$#" -ge 2 ] || { echo "错误: --rerun-max-points 需要参数" >&2; exit 2; }
            RERUN=1
            RERUN_MAX_POINTS="$2"
            shift 2
            ;;
        --rerun-app)
            [ "$#" -ge 2 ] || { echo "错误: --rerun-app 需要参数" >&2; exit 2; }
            RERUN=1
            RERUN_APP="$2"
            shift 2
            ;;
        --rerun-recording-id)
            [ "$#" -ge 2 ] || { echo "错误: --rerun-recording-id 需要参数" >&2; exit 2; }
            RERUN=1
            RERUN_RECORDING_ID="$2"
            shift 2
            ;;
        --rerun-entity-prefix)
            [ "$#" -ge 2 ] || { echo "错误: --rerun-entity-prefix 需要参数" >&2; exit 2; }
            RERUN=1
            RERUN_ENTITY_PREFIX="$2"
            shift 2
            ;;
        --rerun-robot-body)
            RERUN=1
            RERUN_ROBOT_BODY=1
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
        *)
            echo "错误: 未知选项 $1" >&2
            usage
            exit 2
            ;;
    esac
done

[ -n "$PLY_FILE" ] || [ -n "$PCD_FILE" ] || { usage; exit 2; }

# 点云默认跟随 LidarRender 的 local 话题（ZeroSim 四足传 /go2/cloud）。
[ -n "$RERUN_CLOUD_TOPICS" ] || RERUN_CLOUD_TOPICS="$LIDAR_LOCAL_TOPIC"

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
    echo "先运行: bash $ROOT/scripts/build.sh" >&2
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
    -e "QT_X11_NO_MITSHM=1"
    -e "NVIDIA_DRIVER_CAPABILITIES=all"
)
if [ -n "${DISPLAY:-}" ]; then
    DOCKER_ARGS+=(-e "DISPLAY=$DISPLAY" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw")
fi
if [ -n "${ROS_MASTER_URI:-}" ]; then
    DOCKER_ARGS+=(-e "ROS_MASTER_URI=$ROS_MASTER_URI")
fi
DOCKER_ARGS+=(
    -e "ROBOKIT_RERUN=$RERUN"
    -e "ROBOKIT_RERUN_URL=$RERUN_URL"
    -e "ROBOKIT_RERUN_SERVE=$RERUN_SERVE"
    -e "ROBOKIT_RERUN_PORT=$RERUN_PORT"
    -e "ROBOKIT_RERUN_IMAGE_TOPIC=$RERUN_IMAGE_TOPIC"
    -e "ROBOKIT_RERUN_DEPTH_TOPIC=$RERUN_DEPTH_TOPIC"
    -e "ROBOKIT_RERUN_CLOUD_TOPICS=$RERUN_CLOUD_TOPICS"
    -e "ROBOKIT_RERUN_CLOUD_HZ=$RERUN_CLOUD_HZ"
    -e "ROBOKIT_RERUN_ODOM_TOPIC=$RERUN_ODOM_TOPIC"
    -e "ROBOKIT_RERUN_ODOM_HZ=$RERUN_ODOM_HZ"
    -e "ROBOKIT_RERUN_MARKER_TOPICS=$RERUN_MARKER_TOPICS"
    -e "ROBOKIT_RERUN_MARKER_ARRAY_TOPICS=$RERUN_MARKER_ARRAY_TOPICS"
    -e "ROBOKIT_RERUN_MARKER_HZ=$RERUN_MARKER_HZ"
    -e "ROBOKIT_RERUN_MAX_POINTS=$RERUN_MAX_POINTS"
    -e "ROBOKIT_RERUN_APP=$RERUN_APP"
    -e "ROBOKIT_RERUN_RECORDING_ID=$RERUN_RECORDING_ID"
    -e "ROBOKIT_RERUN_ENTITY_PREFIX=$RERUN_ENTITY_PREFIX"
    -e "ROBOKIT_RERUN_ROBOT_BODY=$RERUN_ROBOT_BODY"
    -e "ROBOKIT_RERUN_PATH_TOPICS=$RERUN_PATH_TOPICS"
)
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
        -e "LIDAR_ODOM_TOPIC=${LIDAR_ODOM_TOPIC:-$ODOM_TOPIC}"
        -e "LIDAR_LOCAL_TOPIC=$LIDAR_LOCAL_TOPIC"
        -e "LIDAR_GLOBAL_TOPIC=$LIDAR_GLOBAL_TOPIC"
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
