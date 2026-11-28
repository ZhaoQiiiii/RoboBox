#!/bin/bash
# RoboKit Rerun bridge: one sink (Python 3.10 + rerun) and one ROS tee (Python 3.8).
set -euo pipefail

[ "${ROBOKIT_RERUN:-0}" = "1" ] || exit 0

RUNTIME=/opt/rerun-bridge/runtime
SINK_PY=/opt/rerun-venv/bin/python
URL="${ROBOKIT_RERUN_URL:-rerun+http://127.0.0.1:9877/proxy}"
APP="${ROBOKIT_RERUN_APP:-ZeroAgent}"
RECORDING_ID="${ROBOKIT_RERUN_RECORDING_ID:-}"
ENTITY_PREFIX="${ROBOKIT_RERUN_ENTITY_PREFIX:-/world/robokit}"
ROBOT_BODY="${ROBOKIT_RERUN_ROBOT_BODY:-0}"
IMAGE_TOPIC="${ROBOKIT_RERUN_IMAGE_TOPIC:-/gsplat_render/camera0/color/image/compressed}"
DEPTH_TOPIC="${ROBOKIT_RERUN_DEPTH_TOPIC:-}"
CLOUD_TOPICS="${ROBOKIT_RERUN_CLOUD_TOPICS:-${ROBOKIT_RERUN_CLOUD_TOPIC:-}}"
PATH_TOPICS="${ROBOKIT_RERUN_PATH_TOPICS:-}"
CLOUD_HZ="${ROBOKIT_RERUN_CLOUD_HZ:-5}"
ODOM_TOPIC="${ROBOKIT_RERUN_ODOM_TOPIC:-}"
ODOM_HZ="${ROBOKIT_RERUN_ODOM_HZ:-10}"
MARKER_TOPICS="${ROBOKIT_RERUN_MARKER_TOPICS:-}"
MARKER_ARRAY_TOPICS="${ROBOKIT_RERUN_MARKER_ARRAY_TOPICS:-}"
MARKER_HZ="${ROBOKIT_RERUN_MARKER_HZ:-5}"
MAX_POINTS="${ROBOKIT_RERUN_MAX_POINTS:-20000}"
SERVE="${ROBOKIT_RERUN_SERVE:-0}"
PORT="${ROBOKIT_RERUN_PORT:-9880}"

tee_args=(--sink 127.0.0.1:19084
          --image-topic "$IMAGE_TOPIC"
          --depth-topic "$DEPTH_TOPIC"
          --cloud-hz "$CLOUD_HZ"
          --max-points "$MAX_POINTS")
for spec in $CLOUD_TOPICS; do tee_args+=(--cloud-topic "$spec"); done
for spec in $PATH_TOPICS; do tee_args+=(--path-topic "$spec"); done
if [ -n "$ODOM_TOPIC" ]; then
    tee_args+=(--odom-topic "$ODOM_TOPIC" --odom-hz "$ODOM_HZ")
fi
for topic in $MARKER_TOPICS; do tee_args+=(--marker-topic "$topic"); done
for topic in $MARKER_ARRAY_TOPICS; do tee_args+=(--marker-array-topic "$topic"); done
if [ -n "$MARKER_TOPICS$MARKER_ARRAY_TOPICS" ]; then
    tee_args+=(--marker-hz "$MARKER_HZ")
fi

sink_args=(--listen 127.0.0.1:19084 --app "$APP" --entity-prefix "$ENTITY_PREFIX")
if [ "$ROBOT_BODY" = "1" ]; then
    sink_args+=(--robot-body)
fi
if [ -n "$RECORDING_ID" ]; then
    sink_args+=(--recording-id "$RECORDING_ID")
fi
if [ "$SERVE" = "1" ]; then
    sink_args+=(--serve --serve-port "$PORT")
else
    sink_args+=(--url "$URL")
fi

pids=()
cleanup() {
    trap - EXIT TERM INT
    pkill -f ros_tee.py 2>/dev/null || true
    for pid in "${pids[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT TERM INT

echo "[rerun-bridge] sink: ${sink_args[*]} entity-prefix=$ENTITY_PREFIX robot-body=$ROBOT_BODY"
"$SINK_PY" -u "$RUNTIME/rerun_sink.py" "${sink_args[@]}" &
pids+=("$!")
sleep 0.5

echo "[rerun-bridge] tee: image=${IMAGE_TOPIC:-<off>} depth=${DEPTH_TOPIC:-<off>} clouds=${CLOUD_TOPICS:-<off>} odom=${ODOM_TOPIC:-<off>} markers=${MARKER_TOPICS:-<off>} marker_arrays=${MARKER_ARRAY_TOPICS:-<off>} paths=${PATH_TOPICS:-<off>}"
(
    # rospy exits when the master is not up yet; keep retrying instead of
    # taking the whole bridge down (run.sh starts Roscore before RoboKit).
    while true; do
        if (
            set +u
            source /opt/ros/noetic/setup.bash
            set -u
            exec /usr/bin/python3 -u "$RUNTIME/ros_tee.py" "${tee_args[@]}"
        ); then
            status=0
        else
            status=$?
        fi
        echo "[rerun-bridge] tee exited (status=$status); restarting in 2s"
        sleep 2
    done
) &
pids+=("$!")

# The sink retries its Rerun connection and the tee restarts itself; if the
# sink itself dies the bridge ends and the dispatcher keeps the renderers.
wait -n || true
echo "[rerun-bridge] a process exited; stopping"
