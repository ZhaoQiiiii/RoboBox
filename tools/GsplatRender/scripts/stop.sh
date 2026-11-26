#!/bin/bash
# 停止并删除 GsplatRender 容器。
#
# 用法:
#   bash scripts/stop.sh [容器名]
#
# 容器名默认 gsplat-render，与 scripts/run.sh 的 --name 对应。
set -euo pipefail

NAME="${1:-gsplat-render}"

case "$NAME" in
    -h|--help)
        echo "用法: bash $0 [容器名]，默认 gsplat-render" >&2
        exit 0
        ;;
esac

command -v docker >/dev/null 2>&1 || { echo "错误: 没有可用的 docker" >&2; exit 1; }

if docker container inspect "$NAME" >/dev/null 2>&1; then
    docker stop "$NAME" >/dev/null 2>&1 || true
    docker rm "$NAME" >/dev/null 2>&1 || true
    echo "已停止并删除容器: $NAME"
else
    echo "容器不存在: $NAME"
fi
