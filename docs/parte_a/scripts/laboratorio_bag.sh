#!/usr/bin/env bash
# Helper minimo para procesar el bag tp_final_ws/bags/laboratorio.
#
# Uso tipico, desde la raiz del repo:
#   bash docs/parte_a/scripts/laboratorio_bag.sh info
#   bash docs/parte_a/scripts/laboratorio_bag.sh slam      # terminal 1
#   bash docs/parte_a/scripts/laboratorio_bag.sh play      # terminal 2
#   bash docs/parte_a/scripts/laboratorio_bag.sh map       # terminal 1
#   bash docs/parte_a/scripts/laboratorio_bag.sh play      # terminal 2
#   bash docs/parte_a/scripts/laboratorio_bag.sh localize  # terminal 1
#   bash docs/parte_a/scripts/laboratorio_bag.sh play      # terminal 2

set -euo pipefail

usage() {
  cat <<'EOF'
Uso:
  bash docs/parte_a/scripts/laboratorio_bag.sh info
  bash docs/parte_a/scripts/laboratorio_bag.sh slam
  bash docs/parte_a/scripts/laboratorio_bag.sh map
  bash docs/parte_a/scripts/laboratorio_bag.sh localize
  bash docs/parte_a/scripts/laboratorio_bag.sh play

Variables opcionales:
  BAG_DIR      Ruta al rosbag. Default: tp_final_ws/bags/laboratorio
  ROBOT_NAME   Namespace sin barra. Default: tb4_1
  RUN_ID       Nombre de corrida. Default: laboratorio
  RUN_ROOT     Carpeta de artefactos. Default: runs/$RUN_ID
  BAG_RATE     Velocidad de ros2 bag play. Default: 1.0
  LAUNCH_RVIZ  true/false para slam. Default: false
EOF
}

command="${1:-}"
if [[ -z "${command}" || "${command}" == "-h" || "${command}" == "--help" ]]; then
  usage
  exit 0
fi

if git_root="$(git rev-parse --show-toplevel 2>/dev/null)"; then
  REPO_ROOT="${REPO_ROOT:-${git_root}}"
else
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  REPO_ROOT="${REPO_ROOT:-$(cd "${script_dir}/../../.." && pwd)}"
fi

WS_ROOT="${WS_ROOT:-${REPO_ROOT}/tp_final_ws}"
BAG_DIR="${BAG_DIR:-${WS_ROOT}/bags/laboratorio}"
ROBOT_NAME="${ROBOT_NAME:-tb4_1}"
RUN_ID="${RUN_ID:-laboratorio}"
RUN_ROOT="${RUN_ROOT:-${REPO_ROOT}/runs/${RUN_ID}}"
BAG_RATE="${BAG_RATE:-1.0}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-false}"

TRAJECTORY_FILE="${RUN_ROOT}/parte_a/trajectory.json"
MAP_PREFIX="${RUN_ROOT}/parte_a/map"
MAP_YAML="${MAP_PREFIX}.yaml"

setup_ros() {
  local nounset_was_on=0
  case "$-" in
    *u*) nounset_was_on=1; set +u ;;
  esac
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
  if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WS_ROOT}/install/setup.bash"
  else
    echo "[laboratorio_bag] Aviso: no existe ${WS_ROOT}/install/setup.bash; compila el workspace antes de lanzar nodos." >&2
  fi
  if [[ "${nounset_was_on}" -eq 1 ]]; then
    set -u
  fi
}

require_bag() {
  if [[ ! -f "${BAG_DIR}/metadata.yaml" ]]; then
    echo "[laboratorio_bag] No existe ${BAG_DIR}/metadata.yaml" >&2
    exit 1
  fi
}

prepare_run_dirs() {
  mkdir -p "${RUN_ROOT}/parte_a" "${RUN_ROOT}/logs" "${RUN_ROOT}/config"
}

print_context() {
  cat <<EOF
[laboratorio_bag]
  BAG_DIR=${BAG_DIR}
  ROBOT_NAME=${ROBOT_NAME}
  RUN_ROOT=${RUN_ROOT}
  TRAJECTORY_FILE=${TRAJECTORY_FILE}
  MAP_YAML=${MAP_YAML}
EOF
}

require_bag

case "${command}" in
  info)
    setup_ros
    print_context
    ros2 bag info "${BAG_DIR}"
    if command -v ros2 >/dev/null 2>&1; then
      ros2 run tp_a_slam_aruco check_bag_contract "${BAG_DIR}" \
        --robot-namespace "${ROBOT_NAME}" || true
    fi
    ;;

  play)
    setup_ros
    print_context
    ros2 bag play "${BAG_DIR}" --clock --rate "${BAG_RATE}" \
      --disable-keyboard-controls
    ;;

  slam)
    setup_ros
    prepare_run_dirs
    print_context
    cd "${WS_ROOT}"
    ros2 launch tp_a_slam_aruco parte_a_slam.launch.py \
      robot_namespace:="${ROBOT_NAME}" \
      calibration_file:="${WS_ROOT}/src/tp_a_slam_aruco/config/camera_${ROBOT_NAME}.yaml" \
      trajectory_file:="${TRAJECTORY_FILE}" \
      diagnostics_file:="${RUN_ROOT}/parte_a/aruco_detections.csv" \
      geometry_debug_file:="${RUN_ROOT}/parte_a/aruco_geometry_debug.csv" \
      artifact_dir:="${RUN_ROOT}" \
      run_id:="${RUN_ID}" \
      use_bag_tf:=true \
      use_sim_time:=true \
      launch_rviz:="${LAUNCH_RVIZ}"
    ;;

  map)
    setup_ros
    prepare_run_dirs
    if [[ ! -s "${TRAJECTORY_FILE}" ]]; then
      echo "[laboratorio_bag] Falta ${TRAJECTORY_FILE}; primero corre el subcomando slam y reproduce el bag completo." >&2
      exit 1
    fi
    print_context
    cd "${WS_ROOT}"
    ros2 launch tp_a_slam_aruco parte_a_mapa.launch.py \
      robot_namespace:="${ROBOT_NAME}" \
      trajectory_file:="${TRAJECTORY_FILE}" \
      map_output:="${MAP_PREFIX}" \
      artifact_dir:="${RUN_ROOT}" \
      run_id:="${RUN_ID}" \
      use_bag_tf:=true \
      resolution:=0.05
    ;;

  localize)
    setup_ros
    if [[ ! -s "${MAP_YAML}" ]]; then
      echo "[laboratorio_bag] Falta ${MAP_YAML}; primero genera el mapa." >&2
      exit 1
    fi
    if [[ ! -s "${TRAJECTORY_FILE}" ]]; then
      echo "[laboratorio_bag] Falta ${TRAJECTORY_FILE}; primero corre SLAM." >&2
      exit 1
    fi
    print_context
    cd "${WS_ROOT}"
    ros2 launch tp_b_navigation parte_b.launch.py \
      profile:=bag_tb4 \
      robot_namespace:="${ROBOT_NAME}" \
      map_yaml:="${MAP_YAML}" \
      landmark_map_file:="${TRAJECTORY_FILE}" \
      artifact_dir:="${RUN_ROOT}" \
      run_id:="${RUN_ID}" \
      cmd_vel_topic:=/test/cmd_vel \
      enable_safety_gates:=false \
      launch_rviz:=true
    ;;

  *)
    usage >&2
    exit 2
    ;;
esac
