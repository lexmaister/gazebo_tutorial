#!/usr/bin/env bash
set -euo pipefail

# ---------- Config ----------
# If your real topic is /j7_statel, replace it below.
CTRL_TOPICS=(/j1_ctrl /j2_ctrl /j3_ctrl /j4_ctrl /j5_ctrl /j6_ctrl /j7_ctrl /j_f1_ctrl /j_f2_ctrl)
STATE_TOPICS=(/j1_state /j2_state /j3_state /j4_state /j5_state /j6_state /j7_state /j_f1_state /j_f2_state)

# ---------- Environment ----------
# Source ROS 2 if not already sourced
if [[ -z "${ROS_DISTRO-}" ]]; then
  echo "Source ${ROS_DISTRO-} environment"
  for d in jazzy humble iron rolling; do
    if [[ -f "/opt/ros/$d/setup.bash" ]]; then
      # shellcheck source=/dev/null
      set +u
      source "/opt/ros/$d/setup.bash"
      set -u
      echo "Sourced /opt/ros/$d/setup.bash"
      break
    fi
  done
fi

# ---------- Cleanup on exit ----------
cleanup() {
  echo
  echo "Stopping bridge and rqt..."
  [[ -n "${BRIDGE_PID-}" ]] && kill "${BRIDGE_PID}" 2>/dev/null || true
  [[ -n "${RQT_PID-}"    ]] && kill "${RQT_PID}"    2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ---------- Build bridge args ----------
# Build bridge mappings for controls
BRIDGE_ARGS_CTRL=()
for t in "${CTRL_TOPICS[@]}"; do
  BRIDGE_ARGS_CTRL+=("${t}@std_msgs/msg/Float64@gz.msgs.Double")
done

# Build bridge mappings for joint state topics (Model to JointState)
BRIDGE_ARGS_STATE=()
for t in "${STATE_TOPICS[@]}"; do
  BRIDGE_ARGS_STATE+=("${t}@sensor_msgs/msg/JointState@gz.msgs.Model")
done

# Combine both arrays for the launch
BRIDGE_ARGS=( "${BRIDGE_ARGS_CTRL[@]}" "${BRIDGE_ARGS_STATE[@]}" )

echo "Starting ros_gz_bridge with ${#BRIDGE_ARGS_CTRL[@]} control mappings and ${#BRIDGE_ARGS_STATE[@]} state mappings..."
printf '  CTRL : %s\n' "${CTRL_TOPICS[@]}"
printf '  STATE: %s\n' "${STATE_TOPICS[@]}"

# ---------- Run bridge ----------
ros2 run ros_gz_bridge parameter_bridge "${BRIDGE_ARGS[@]}" &
BRIDGE_PID=$!
echo "ros_gz_bridge PID: ${BRIDGE_PID}"

# ---------- Start rqt gui (no perspective) ----------
echo "Starting rqt_gui..."
ros2 run rqt_gui rqt_gui --clear-config &
RQT_PID=$!
echo "rqt_gui PID: ${RQT_PID}"

echo
echo "Bridge and rqt running. Press Ctrl+C to stop."
wait