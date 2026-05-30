#!/usr/bin/env bash
set -euo pipefail

BAG_DIR="${1:-$HOME/rosbags/quadrotor_mpc}"
TAG="${2:-hover_debug}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_FILE="${BAG_DIR}/${TAG}_${STAMP}.bag"

TOPICS=(
  /vrpn_client_node/NMPC0/pose
  /odom
  /mpc_ref_traj
  /mavros/state
  /mavros/setpoint_raw/attitude
  /mavros/local_position/pose
  /mavros/local_position/odom
  /mavros/local_position/velocity_local
  /mavros/imu/data
  /mavros/battery
  /mpc_debug/mode
  /mpc_debug/ref_pose
  /mpc_debug/raw_control
  /mpc_debug/acado_pred_path
  /mpc_debug/acado_ref_path
  /mpc_debug/acado_pred_u
  /mpc_debug/acado_x0
  /mpc_debug/hover_thrust
)

mkdir -p "${BAG_DIR}"

if ! command -v rosbag >/dev/null 2>&1; then
  echo "[ERROR] 未找到 rosbag，请先 source ROS 环境。"
  exit 1
fi

echo "[INFO] 输出文件: ${BAG_FILE}"
echo "[INFO] 录制以下 topic:"
printf '  - %s\n' "${TOPICS[@]}"
echo "[INFO] 按 Ctrl+C 停止录制"
echo "[INFO] 提示：可以通过 RViz 添加 Path 显示 /mpc_debug/acado_pred_path 和 /mpc_debug/acado_ref_path 直观分析轨迹。"

rosbag record --tcpnodelay -O "${BAG_FILE}" "${TOPICS[@]}"