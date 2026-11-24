#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLY_FILE="${1:?用法: scripts/run.sh <ply_file> [--enable_gimbal]}"
shift
exec python3 -u "$ROOT/renderer/gsplat_renderer_ros1.py" \
    --ply_file "$PLY_FILE" \
    --params_file "$ROOT/config/renderer_params.yaml" "$@"
