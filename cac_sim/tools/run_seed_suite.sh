#!/usr/bin/env bash
# Multi-seed end-to-end validation (TEST TOOL).
#
#   tools/run_seed_suite.sh <out_dir> <seed> [<seed> ...]
#
# For each seed: start a FRESH simulation (headless Gazebo server, autonomy on,
# no RViz) in its own process group, run tools/gt_monitor.py until the mission
# reaches COMPLETE/FAILED (or the timeout), stop everything, and keep the launch
# log, the per-sample CSV and the summary JSON in <out_dir>. Finally prints a
# table (tools/suite_report.py).
#
# Needs a display for the rendering sensors (DISPLAY, default :0). Uses
# loopback-only Gazebo transport / ROS discovery, like the launch file.
# SUITE_RESUME=1 skips seeds that already have a seed_N_summary.json in <out_dir>
# (rerun only the missing ones after an interruption).
# (no `set -u`: ROS setup.bash reads unset variables)
OUT=$1; shift
mkdir -p "$OUT"
HERE=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/jazzy/setup.bash
source "$(cd "$HERE/../../../.." && pwd)/setup.bash" 2>/dev/null || true
export DISPLAY=${DISPLAY:-:0} GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
TIMEOUT=${SUITE_TIMEOUT:-900}

cleanup() {
  [ -n "${1:-}" ] && kill -INT -- -"$1" 2>/dev/null && sleep 8
  [ -n "${1:-}" ] && kill -KILL -- -"$1" 2>/dev/null
  # stray processes from a crashed run would publish a second /clock into the next one
  for pat in parameter_bridge "gz sim" "ruby /opt/ros" "lib/cac_sim/" "lib/nav2_" ekf_node robot_state_publisher; do
    pkill -9 -f -- "$pat" 2>/dev/null
  done
  sleep 2
}

for SEED in "$@"; do
  if [ "${SUITE_RESUME:-0}" = 1 ] && [ -s "$OUT/seed_${SEED}_summary.json" ]; then
    echo "=== seed $SEED: summary exists, skipped (SUITE_RESUME=1)"
    continue
  fi
  echo "=== seed $SEED ($(date +%T))"
  cleanup ""
  setsid ros2 launch cac_sim cac_sim.launch.py seed:="$SEED" headless:=true use_rviz:=false \
      > "$OUT/launch_$SEED.log" 2>&1 < /dev/null &
  sleep 2
  PG=$(ps -eo pgid,args --no-headers | awk -v s="seed:=$SEED" 'index($0, "ros2 launch cac_sim") && index($0, s) && !/awk/ {print $1; exit}')
  timeout $((TIMEOUT + 60)) python3 "$HERE/gt_monitor.py" --seed "$SEED" --duration "$TIMEOUT" \
      --out "$OUT/seed_$SEED" > "$OUT/monitor_$SEED.txt" 2>&1
  cleanup "$PG"
done
python3 "$HERE/suite_report.py" "$OUT"
