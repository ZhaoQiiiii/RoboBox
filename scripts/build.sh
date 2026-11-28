#!/bin/bash
# 构建 RoboKit 统一工具镜像 robokit:latest
#
# 用法:
#   bash scripts/build.sh [--no-proxy] [--pull]
#
# 环境变量:
#   UBUNTU_MIRROR          默认 mirrors.tuna.tsinghua.edu.cn
#   ROS_APT_MIRROR         默认 https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu
#   HTTP_PROXY/HTTPS_PROXY 构建期代理；未设置时尝试 127.0.0.1:7890

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="robokit:latest"
USE_PROXY=1
PULL=0

usage() {
    echo "用法: bash $0 [--no-proxy] [--pull]" >&2
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --no-proxy)
            USE_PROXY=0
            shift
            ;;
        --pull)
            PULL=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "错误: 未知参数 $1" >&2
            usage
            exit 2
            ;;
    esac
done

command -v docker >/dev/null 2>&1 || { echo "错误: 没有可用的 docker" >&2; exit 1; }
docker info >/dev/null 2>&1 || { echo "错误: docker daemon 不可用" >&2; exit 1; }
[ -f "$ROOT/docker/Dockerfile" ] || { echo "错误: 找不到 $ROOT/docker/Dockerfile" >&2; exit 1; }

PROXY_FROM_ENV=0
if [ -n "${HTTP_PROXY+x}${http_proxy+x}${HTTPS_PROXY+x}${https_proxy+x}" ]; then
    PROXY_FROM_ENV=1
fi
HTTP_PROXY_ARG="${HTTP_PROXY-${http_proxy-http://127.0.0.1:7890}}"
HTTPS_PROXY_ARG="${HTTPS_PROXY-${https_proxy-http://127.0.0.1:7890}}"

proxy_reachable() {
    local hostport="${1#*://}"
    hostport="${hostport%%/*}"
    timeout 3 bash -c "exec 3<>/dev/tcp/${hostport%:*}/${hostport##*:}" 2>/dev/null
}

PROXY_ARGS=()
if [ "$USE_PROXY" -eq 1 ] && [ -n "$HTTP_PROXY_ARG" ]; then
    if proxy_reachable "$HTTP_PROXY_ARG"; then
        PROXY_ARGS+=(
            --build-arg "HTTP_PROXY=$HTTP_PROXY_ARG"
            --build-arg "HTTPS_PROXY=$HTTPS_PROXY_ARG"
            --build-arg "http_proxy=$HTTP_PROXY_ARG"
            --build-arg "https_proxy=$HTTPS_PROXY_ARG"
        )
        echo "构建代理: $HTTP_PROXY_ARG"
    elif [ "$PROXY_FROM_ENV" -eq 1 ]; then
        echo "错误: 代理 $HTTP_PROXY_ARG 连不上；修正环境变量或加 --no-proxy" >&2
        exit 1
    else
        echo "提示: 默认代理 $HTTP_PROXY_ARG 不可达，改为无代理构建" >&2
    fi
fi

BUILD_ARGS=()
[ -n "${UBUNTU_MIRROR:-}" ] && BUILD_ARGS+=(--build-arg "UBUNTU_MIRROR=$UBUNTU_MIRROR")
[ -n "${ROS_APT_MIRROR:-}" ] && BUILD_ARGS+=(--build-arg "ROS_APT_MIRROR=$ROS_APT_MIRROR")
[ -n "${TORCH_CUDA_ARCH_LIST:-}" ] && BUILD_ARGS+=(--build-arg "TORCH_CUDA_ARCH_LIST=$TORCH_CUDA_ARCH_LIST")
[ "$PULL" -eq 1 ] && BUILD_ARGS+=(--pull)

echo "镜像: $IMAGE"
echo "上下文: $ROOT"

docker build \
    --network host \
    -f "$ROOT/docker/Dockerfile" \
    -t "$IMAGE" \
    "${BUILD_ARGS[@]}" \
    "${PROXY_ARGS[@]}" \
    "$ROOT"
