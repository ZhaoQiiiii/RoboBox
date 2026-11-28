#!/bin/bash
# robokit:latest 入口分发：gsplat、lidar 或 both。
set -euo pipefail

start_rerun_bridge() {
    [ "${ROBOKIT_RERUN:-0}" = "1" ] || return 0
    [ -e /tmp/robokit-rerun.started ] && return 0
    touch /tmp/robokit-rerun.started
    log_dir="${ROBOKIT_LOG_DIR:-/logs}"
    if [ -d "$log_dir" ] && [ -w "$log_dir" ]; then
        /opt/rerun-bridge/docker/entrypoint.sh >>"$log_dir/rerun_bridge.log" 2>&1 &
    else
        /opt/rerun-bridge/docker/entrypoint.sh &
    fi
}

start_rerun_bridge

tool="${1:-}"
case "$tool" in
    gsplat)
        shift
        exec /opt/gsplat-render/docker/entrypoint.sh "$@"
        ;;
    lidar)
        shift
        exec /opt/lidar-render/docker/entrypoint.sh "$@"
        ;;
    both)
        shift
        ply="${1:-}"
        pcd="${2:-}"
        if [ -z "$ply" ] || [ -z "$pcd" ]; then
            echo "用法: both <ply 文件> <pcd 文件>" >&2
            exit 2
        fi

        log_dir="${ROBOKIT_LOG_DIR:-/logs}"
        gsplat_pid=""
        lidar_pid=""

        run_tool() {
            local log_name="$1"
            shift
            if [ -d "$log_dir" ] && [ -w "$log_dir" ]; then
                # exec 后子 shell 变成工具进程，$! 仍是工具 PID，信号能直达。
                ( exec > >(tee -a "$log_dir/$log_name") 2>&1; exec "$@" ) &
            else
                "$@" &
            fi
        }

        cleanup() {
            local status=$?
            trap - EXIT TERM INT
            pkill -f rerun_sink.py 2>/dev/null || true
            pkill -f ros_tee.py 2>/dev/null || true
            for pid in "$lidar_pid" "$gsplat_pid"; do
                [ -n "$pid" ] && kill -TERM "$pid" 2>/dev/null || true
            done
            wait 2>/dev/null || true
            exit "$status"
        }
        trap cleanup EXIT TERM INT

        run_tool gsplat_renderer.log /opt/gsplat-render/docker/entrypoint.sh "$ply"
        gsplat_pid=$!
        run_tool lidar_render.log /opt/lidar-render/docker/entrypoint.sh "$pcd"
        lidar_pid=$!

        status=0
        wait -n || status=$?
        exit "$status"
        ;;
    *)
        echo "用法: <gsplat|lidar|both> <scene 文件> [参数]" >&2
        exit 2
        ;;
esac
