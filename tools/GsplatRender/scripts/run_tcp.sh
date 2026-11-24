#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLY_FILE="${1:?用法: scripts/run_tcp.sh <ply_file> [--enable_gimbal] [--host host] [--pose-port port] [--frame-port port]}"
shift
exec python3 -u "$ROOT/renderer/gsplat_renderer_tcp.py" \
    --ply_file "$PLY_FILE" \
    --params_file "$ROOT/config/renderer_params.yaml" "$@"
