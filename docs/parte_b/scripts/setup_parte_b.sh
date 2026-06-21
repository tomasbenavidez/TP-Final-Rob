#!/bin/zsh
# setup_parte_b.sh — Prepara el entorno para correr Parte B en Mac.
#
#   source docs/parte_b/scripts/setup_parte_b.sh
#
# Hace lo que `source install/setup.bash` debería hacer, pero a mano, porque en
# este Mac (shell zsh + overlay RoboStack) el setup.bash/zsh de colcon falla
# (errores de compdef / rutas). Además fija CycloneDDS en loopback (ver el .xml).
#
# Ajustar estas 3 rutas si la instalación difiere:

CONDA_SH="$HOME/miniforge3/etc/profile.d/conda.sh"
ROSENV="rosenv"
WS_PROF="$HOME/ws/install"                                   # overlay del profe (turtlebot3_custom_simulation)
WS_TPB="$HOME/Documents/GitHub/TP-Final-Rob/tp_final_ws"     # nuestro workspace
PYVER="python3.11"
HERE="${0:A:h}"                                              # carpeta de este script

setopt NULL_GLOB 2>/dev/null || true
source "$CONDA_SH"
conda activate "$ROSENV"

# --- AMENT: rosenv + cada prefijo del overlay del profe + nuestro paquete ---
AP="$AMENT_PREFIX_PATH"
for d in "$WS_PROF"/*/; do
  [ -d "${d}share" ] && AP="${d%/}:$AP"
done
AP="$WS_TPB/install/tp_b_navigation:$AP"
export AMENT_PREFIX_PATH="$AP"

# --- PYTHONPATH: site-packages de cada paquete + egg-info del build (para `ros2 run`) ---
PP="$PYTHONPATH"
for d in "$WS_PROF"/*/lib/$PYVER/site-packages "$WS_TPB"/install/*/lib/$PYVER/site-packages; do
  [ -d "$d" ] && PP="$d:$PP"
done
# El install es egg-link: el metadata de los entry points está en build/.
PP="$WS_TPB/build/tp_b_navigation:$PP"
export PYTHONPATH="$PP"

# --- PATH: ejecutables de cada paquete ---
for d in "$WS_PROF"/*/lib/* "$WS_TPB"/install/*/lib/*; do
  [ -d "$d" ] && case ":$PATH:" in *":$d:"*) ;; *) export PATH="$d:$PATH";; esac
done

# --- TurtleBot3 + Gazebo ---
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH="$WS_PROF/turtlebot3_custom_simulation/share/turtlebot3_custom_simulation/models:$GAZEBO_MODEL_PATH"

# --- DDS en loopback (imprescindible en este Mac, ver cyclonedds_loopback.xml) ---
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HERE/cyclonedds_loopback.xml"

echo "[setup_parte_b] listo. RMW=$RMW_IMPLEMENTATION  TB3=$TURTLEBOT3_MODEL"
echo "[setup_parte_b] paquetes: $(ros2 pkg prefix tp_b_navigation 2>/dev/null && echo OK || echo 'FALTA build')"
