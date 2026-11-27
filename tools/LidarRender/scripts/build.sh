#!/bin/bash
# 构建统一镜像 robokit:latest（含 GsplatRender 与 LidarRender）。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$ROOT/../../docker/build.sh" "$@"
