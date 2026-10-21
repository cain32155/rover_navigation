#!/usr/bin/env bash
# Run one command against a fresh simulation (TEST TOOL).
#
#   tools/with_sim.sh <out_dir> <timeout_s> "<launch args>" <command...>
#   e.g. tools/with_sim.sh $WS/suite_results/x 300 "randomize:=false autonomy:=false" \
#            python3 tools/marker_range_test.py --out $WS/suite_results/x/r1
#
# Starts `ros2 launch cac_sim cac_sim.launch.py headless:=true use_rviz:=false <launch args>`
# in its own process group (log: <out_dir>/launch.log, run dir <out_dir>/run), waits
# --warmup seconds (WARMUP, default 15), runs the command with a timeout, then stops
# the launch group and any stray simulation processes (same patterns as
# run_seed_suite.sh). Never run two of these (or a suite) at the same time.
OUT=$1; TMO=$2; LARGS=$3; shift 3
mkdir -p "$OUT"
source /opt/ros/jazzy/setup.bash
HERE=$(cd "$(dirname "$0")" && pwd)
source "$(cd "$HERE/../../../.." && pwd)/setup.bash" 2>/dev/null || true
export DISPLAY=${DISPLAY:-:0} GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
stop_sim() {
  [ -n "${1:-}" ] && kill -INT -- -"$1" 2>/dev/null && sleep 8
  [ -n "${1:-}" ] && kill -KILL -- -"$1" 2>/dev/null
  for pat in parameter_bridge "gz sim" "ruby /opt/ros" "lib/cac_sim/" "lib/nav2_" ekf_node robot_state_publisher rviz2; do
    pkill -9 -f -- "$pat" 2>/dev/null
  done
  sleep 2
}
stop_sim ""
# shellcheck disable=SC2086
setsid ros2 launch cac_sim cac_sim.launch.py headless:=true use_rviz:=false run_dir:="$OUT/run" $LARGS \
    > "$OUT/launch.log" 2>&1 < /dev/null &
LPID=$!
sleep 2
PG=$(ps -o pgid= -p "$LPID" | tr -d ' ')
sleep "${WARMUP:-15}"
timeout "$TMO" "$@"
RC=$?
stop_sim "$PG"
echo "with_sim: command exit $RC"
exit $RC
