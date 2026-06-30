# TP Final de Robótica Autónoma — guía de laboratorio TB4

Esta es la guía única para ejecutar la prueba física con un TurtleBot4. El
flujo es secuencial:

1. **Parte A:** grabar un bag, ejecutar Graph SLAM con ArUco y generar el mapa.
2. **Parte B:** localizar el robot con MCL y navegar sobre ese mapa.
3. **Parte C:** explorar, detectar el cono rojo con RGB-D y aproximarse.

Parte A produce `trajectory.json`, `map.yaml` y `map.pgm`; Partes B y C usan
esos mismos artefactos. No ejecutar las partes simultáneamente.

La preparación local y con bags está verificada. Los gates físicos B1–B3 y
C1–C3 siguen pendientes hasta ejecutarlos sobre un TB4. La evidencia disponible
está en [Parte A](docs/parte_a/tb4-map-comparison.md) y
[Parte B](docs/parte_b/tb4-mcl-obstacle-diagnostic.md).

## 1. Preparar la notebook

Requisitos: notebook Ubuntu del laboratorio con ROS 2 Humble instalado en
`/opt/ros/humble`, `colcon`, GTSAM, OpenCV contrib, SciPy, acceso SSH al TB4 y
el paquete de teleoperación:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

Abrir una terminal en la raíz del repositorio. Cambiar solamente
`ROBOT_NAME` si se usa `tb4_1`. Este bloque compila el workspace y guarda todas
las variables de la corrida en `/tmp/tb4_lab_env.sh`.

```bash
# [Notebook — ejecutar desde la raíz del repositorio]
export ROBOT_NAME=tb4_0
export REPO_ROOT="$(git rev-parse --show-toplevel)"
export WS_ROOT="${REPO_ROOT}/tp_final_ws"
export ROBOT_NS="/${ROBOT_NAME}"
export TB4_SSH_HOST="${ROBOT_NAME}"
export RUN_ID="$(date +%Y%m%d-%H%M)-${ROBOT_NAME}"
export RUN_ROOT="${HOME}/tb4_laboratorio_runs/${RUN_ID}"
export BAG_DIR="${RUN_ROOT}/acquisition/laberinto"
export TRAJECTORY_FILE="${RUN_ROOT}/parte_a/trajectory.json"
export MAP_PREFIX="${RUN_ROOT}/parte_a/map"
export MAP_YAML="${MAP_PREFIX}.yaml"
export MAP_PGM="${MAP_PREFIX}.pgm"
export ODOM_TOPIC="${ROBOT_NS}/odom"
export SCAN_TOPIC="${ROBOT_NS}/scan"
export TF_TOPIC="${ROBOT_NS}/tf"
export TF_STATIC_TOPIC="${ROBOT_NS}/tf_static"
export RGB_TOPIC="${ROBOT_NS}/oakd/rgb/preview/image_raw"
export CAMERA_INFO_TOPIC="${ROBOT_NS}/oakd/rgb/preview/camera_info"
export CMD_VEL_TOPIC="${ROBOT_NS}/cmd_vel"
export DEPTH_TOPIC=""

source /opt/ros/humble/setup.bash

mkdir -p "${RUN_ROOT}/acquisition" "${RUN_ROOT}/parte_a" \
  "${RUN_ROOT}/config" "${RUN_ROOT}/logs"

cat > /tmp/tb4_lab_env.sh <<EOF
export ROBOT_NAME="${ROBOT_NAME}"
export REPO_ROOT="${REPO_ROOT}"
export WS_ROOT="${WS_ROOT}"
export ROBOT_NS="${ROBOT_NS}"
export TB4_SSH_HOST="${TB4_SSH_HOST}"
export RUN_ID="${RUN_ID}"
export RUN_ROOT="${RUN_ROOT}"
export BAG_DIR="${BAG_DIR}"
export TRAJECTORY_FILE="${TRAJECTORY_FILE}"
export MAP_PREFIX="${MAP_PREFIX}"
export MAP_YAML="${MAP_YAML}"
export MAP_PGM="${MAP_PGM}"
export ODOM_TOPIC="${ODOM_TOPIC}"
export SCAN_TOPIC="${SCAN_TOPIC}"
export TF_TOPIC="${TF_TOPIC}"
export TF_STATIC_TOPIC="${TF_STATIC_TOPIC}"
export RGB_TOPIC="${RGB_TOPIC}"
export CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC}"
export CMD_VEL_TOPIC="${CMD_VEL_TOPIC}"
export DEPTH_TOPIC=""
source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/setup.bash"
EOF

cd "${WS_ROOT}"
colcon build --packages-select tp_platform tp_interfaces tp_a_slam_aruco \
  tp_b_navigation tp_c_mission turtlebot3_custom_simulation
source install/setup.bash
echo "RUN_ID=${RUN_ID}"
echo "Artefactos: ${RUN_ROOT}"
```

En **cada terminal nueva de la notebook**, comenzar con una sola línea:

```bash
source /tmp/tb4_lab_env.sh
```

## 2. Preflight: red, sensores y parada

No iniciar drivers desde la notebook: se usan los drivers que ya corren en el
TB4. Confirmar que el namespace seleccionado tiene datos.

```bash
# [Notebook]
source /tmp/tb4_lab_env.sh

ros2 topic echo --once "${ODOM_TOPIC}" nav_msgs/msg/Odometry
ros2 topic echo --once "${SCAN_TOPIC}" sensor_msgs/msg/LaserScan
ros2 topic echo --once "${RGB_TOPIC}" sensor_msgs/msg/Image
ros2 topic echo --once "${CAMERA_INFO_TOPIC}" sensor_msgs/msg/CameraInfo
ros2 topic echo --once "${TF_TOPIC}" tf2_msgs/msg/TFMessage
ros2 topic echo --once "${TF_STATIC_TOPIC}" tf2_msgs/msg/TFMessage
ros2 topic info "${CMD_VEL_TOPIC}" -v
ros2 topic pub --once "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist '{}'
```

**No seguir** si falta algún sensor o TF, si aparecen tópicos del otro TB4, si
hay un publisher de velocidad inesperado o si no funciona la parada manual.

Para Parte C real, chequear además si la OAK-D publica depth. Esto no es
necesario para Parte A/B, pero sí para que el detector de cono pueda estimar la
posición del cono en `map`.

```bash
# [Notebook — buscar depth para Parte C]
source /tmp/tb4_lab_env.sh

ros2 topic list | grep -Ei 'oak|depth|stereo|disparity|aligned|image_raw|camera_info'
```

Si aparece un tópico candidato de depth alineado al RGB, definirlo y confirmar
que publica una imagen con dimensiones compatibles:

```bash
# [Notebook — reemplazar por el tópico real encontrado]
export DEPTH_TOPIC="/${ROBOT_NAME}/RUTA/DEPTH_ALINEADO"
printf 'export DEPTH_TOPIC=%q\n' "${DEPTH_TOPIC}" >> /tmp/tb4_lab_env.sh

ros2 topic echo --once "${RGB_TOPIC}" sensor_msgs/msg/Image \
  | grep -E 'height:|width:|encoding:'
ros2 topic echo --once "${DEPTH_TOPIC}" sensor_msgs/msg/Image \
  | grep -E 'height:|width:|encoding:'
```

Si no aparece ningún tópico de depth, continuar con Parte A/B pero no iniciar
Parte C real: `vision_ready` debe quedar en `false` hasta tener RGB-D alineado.

## 3. Grabar el bag en el TB4

El bag se graba onboard para no depender del transporte de RGB por Wi-Fi. El
siguiente comando crea la carpeta remota y comienza a grabar. Mantener esta
terminal abierta durante el recorrido.

```bash
# [Notebook — Terminal de grabación]
source /tmp/tb4_lab_env.sh

ssh "${TB4_SSH_HOST}" \
  "mkdir -p tb4_laboratorio_runs/${RUN_ID}/acquisition"

ssh -t "${TB4_SSH_HOST}" "bash -lc 'source /opt/ros/humble/setup.bash && ros2 bag record \
  ${ODOM_TOPIC} \
  ${SCAN_TOPIC} \
  ${RGB_TOPIC} \
  ${CAMERA_INFO_TOPIC} \
  ${TF_TOPIC} \
  ${TF_STATIC_TOPIC} \
  ${ROBOT_NS}/imu \
  -o tb4_laboratorio_runs/${RUN_ID}/acquisition/laberinto'"
```

En otra terminal de la notebook, teleoperar a velocidad reducida. El remap es
importante: evita publicar por accidente en `/cmd_vel` global y manda comandos
al TB4 seleccionado.

```bash
# [Notebook — Terminal de teleop]
source /tmp/tb4_lab_env.sh

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:="${CMD_VEL_TOPIC}"
```

Durante la grabación, cubrir el laberinto despacio y reobservar los ArUco desde
varios ángulos. Detener físicamente el robot y recién entonces presionar
`Ctrl+C` en la terminal de grabación. Esperar a que rosbag cierre
`metadata.yaml`.

```bash
# [Notebook — validar y copiar el bag]
source /tmp/tb4_lab_env.sh

ssh "${TB4_SSH_HOST}" \
  "test -s tb4_laboratorio_runs/${RUN_ID}/acquisition/laberinto/metadata.yaml"

mkdir -p "${BAG_DIR}"
rsync -av --progress \
  "${TB4_SSH_HOST}:tb4_laboratorio_runs/${RUN_ID}/acquisition/laberinto/" \
  "${BAG_DIR}/"

ros2 bag info "${BAG_DIR}" | tee "${RUN_ROOT}/config/bag-info.txt"
ros2 run tp_a_slam_aruco check_bag_contract "${BAG_DIR}" \
  --robot-namespace "${ROBOT_NAME}"
```

**Esperar:** `Parte A bag contract OK`. Si falta un tópico, no ejecutar Parte A.

## 4. Parte A — pasada 1: Graph SLAM

Primero iniciar el launch y después reproducir el bag.

```bash
# [Notebook — Terminal 1: Graph SLAM]
source /tmp/tb4_lab_env.sh

ros2 launch tp_a_slam_aruco parte_a_slam.launch.py \
  robot_namespace:="${ROBOT_NAME}" \
  artifact_dir:="${RUN_ROOT}" \
  run_id:="${RUN_ID}" \
  trajectory_file:="${TRAJECTORY_FILE}" \
  use_bag_tf:=true \
  use_sim_time:=true
```

```bash
# [Notebook — Terminal 2: reproducir bag]
source /tmp/tb4_lab_env.sh

ros2 bag play "${BAG_DIR}" --clock --rate 2.0 --disable-keyboard-controls
```

Cuando termine el bag, presionar `Ctrl+C` en Terminal 1 para guardar el JSON.

```bash
# [Notebook — comprobar salida]
source /tmp/tb4_lab_env.sh
test -s "${TRAJECTORY_FILE}"
test -s "${RUN_ROOT}/config/platform-parte-a-slam.yaml"
```

**Esperar:** trayectoria continua, landmarks razonables y `map -> odom` sin
saltos severos en RViz. Si el JSON está vacío o la trayectoria diverge, no
generar el mapa.

Diagnósticos útiles de esta pasada:

- `/aruco_detections`: detecciones ArUco crudas en frame de cámara;
- `/landmarks`: landmarks ArUco optimizados por Graph SLAM;
- `/aruco_base_debug`: puntos de depuración transformados hacia `base_link`;
- CSV de diagnóstico, por defecto `/tmp/aruco_detections.csv` y
  `/tmp/aruco_geometry_debug.csv` si no se pasan rutas explícitas.

En el flujo de laboratorio el JSON se guarda en `${TRAJECTORY_FILE}`. Los
defaults históricos de desarrollo eran `/tmp/trayectoria.json` para la
trayectoria y `/tmp/mapa.yaml` para el mapa; se mantienen como referencia para
entender tests y documentación vieja, pero no son los nombres recomendados para
la corrida física.

## 5. Parte A — pasada 2: mapa

Usar exactamente el mismo bag y `trajectory.json`.

```bash
# [Notebook — Terminal 1: generar mapa]
source /tmp/tb4_lab_env.sh

ros2 launch tp_a_slam_aruco parte_a_mapa.launch.py \
  robot_namespace:="${ROBOT_NAME}" \
  artifact_dir:="${RUN_ROOT}" \
  run_id:="${RUN_ID}" \
  trajectory_file:="${TRAJECTORY_FILE}" \
  map_output:="${MAP_PREFIX}" \
  use_bag_tf:=true
```

```bash
# [Notebook — Terminal 2: reproducir bag]
source /tmp/tb4_lab_env.sh

ros2 bag play "${BAG_DIR}" --clock --rate 2.0 --disable-keyboard-controls
```

Cuando termine el bag, presionar `Ctrl+C` en Terminal 1 para exportar el mapa.

```bash
# [Notebook — comprobar salida]
source /tmp/tb4_lab_env.sh
test -s "${MAP_YAML}"
test -s "${MAP_PGM}"
test -s "${RUN_ROOT}/config/platform-parte-a-mapa.yaml"
```

## 6. Inspeccionar el mapa

```bash
# [Notebook — Terminal 1: map server]
source /tmp/tb4_lab_env.sh

ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="${MAP_YAML}"
```

```bash
# [Notebook — Terminal 2: activar mapa y abrir RViz]
source /tmp/tb4_lab_env.sh

ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
rviz2
```

En RViz usar `Fixed Frame: map` y agregar `/map`. **No seguir** si faltan
paredes, hay paredes dobles severas, se cerraron aberturas transitables o
aparecen conexiones falsas.

Cerrar el map server y RViz antes de Parte B.

## 7. Parte B — localización sin movimiento

La primera ejecución remapea la velocidad a `/test/cmd_vel`; no puede comandar
el robot físico.

```bash
# [Notebook — Terminal 1: Parte B segura]
source /tmp/tb4_lab_env.sh

ros2 launch tp_b_navigation parte_b.launch.py \
  profile:=real_tb4 \
  robot_namespace:="${ROBOT_NAME}" \
  artifact_dir:="${RUN_ROOT}" \
  run_id:="${RUN_ID}" \
  map_yaml:="${MAP_YAML}" \
  landmark_map_file:="${TRAJECTORY_FILE}" \
  cmd_vel_topic:=/test/cmd_vel \
  enable_safety_gates:=true
```

En RViz publicar una pose inicial con **2D Pose Estimate**. Verificar:

```bash
# [Notebook — Terminal 2: checks B1]
source /tmp/tb4_lab_env.sh

ros2 topic echo --once /mcl_pose
ros2 topic echo --once /obstacle_monitor_healthy std_msgs/msg/Bool
ros2 node info /state_machine
ros2 topic info "${CMD_VEL_TOPIC}" -v
```

**Esperar:** nube concentrada, MCL fresco, monitor `true`, `/test/cmd_vel` en
`state_machine` y ningún publisher nuevo sobre `${CMD_VEL_TOPIC}`.

## 8. Parte B — un goal corto

Detener el launch anterior con `Ctrl+C`, confirmar que el robot está quieto y
probar nuevamente la parada manual.

```bash
# [Notebook — publicar cero antes de habilitar movimiento]
source /tmp/tb4_lab_env.sh
ros2 topic pub --once "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist '{}'
```

```bash
# [Notebook — Terminal 1: Parte B con salida física]
source /tmp/tb4_lab_env.sh

ros2 launch tp_b_navigation parte_b.launch.py \
  profile:=real_tb4 \
  robot_namespace:="${ROBOT_NAME}" \
  artifact_dir:="${RUN_ROOT}" \
  run_id:="${RUN_ID}" \
  map_yaml:="${MAP_YAML}" \
  landmark_map_file:="${TRAJECTORY_FILE}" \
  cmd_vel_topic:="${CMD_VEL_TOPIC}" \
  enable_safety_gates:=true
```

Publicar nuevamente la pose inicial y enviar primero un único **2D Goal Pose**
cercano, libre y despejado. Mantener la parada accesible. No probar múltiples
goals ni obstáculos hasta que este recorrido sea seguro.

## 9. Parte C — RGB-D y misión

Detener Parte B, publicar velocidad cero y mantener el robot quieto. Buscar los
tópicos depth disponibles:

```bash
# [Notebook — identificar depth alineado]
source /tmp/tb4_lab_env.sh

ros2 topic pub --once "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist '{}'
ros2 topic list | grep -i depth
ros2 topic list | grep -E 'stereo|aligned'
```

Definir el tópico que corresponda a **depth métrico alineado con el RGB**. Esta
es la única línea que debe completarse durante el laboratorio:

```bash
# [Notebook — reemplazar por el tópico alineado real]
export DEPTH_TOPIC="/${ROBOT_NAME}/RUTA/DEPTH_ALINEADO"
printf 'export DEPTH_TOPIC=%q\n' "${DEPTH_TOPIC}" >> /tmp/tb4_lab_env.sh

ros2 topic echo --once "${RGB_TOPIC}" sensor_msgs/msg/Image \
  | grep -E 'height:|width:|encoding:'
ros2 topic echo --once "${DEPTH_TOPIC}" sensor_msgs/msg/Image \
  | grep -E 'height:|width:|encoding:'
```

RGB y depth deben tener las mismas dimensiones, timestamps frescos y el mismo
campo visual. **No lanzar Parte C** si no se puede demostrar la alineación.

```bash
# [Notebook — Terminal 1: Parte C real]
source /tmp/tb4_lab_env.sh

ros2 launch tp_c_mission parte_c_real.launch.py \
  profile:=real_tb4 \
  robot_namespace:="${ROBOT_NAME}" \
  artifact_dir:="${RUN_ROOT}" \
  run_id:="${RUN_ID}" \
  map_yaml:="${MAP_YAML}" \
  landmark_map_file:="${TRAJECTORY_FILE}" \
  depth_topic:="${DEPTH_TOPIC}" \
  enable_safety_gates:=true
```

Antes de iniciar la misión:

```bash
# [Notebook — Terminal 2: readiness y control de misión]
source /tmp/tb4_lab_env.sh

ros2 topic echo --once /red_cone/vision_ready std_msgs/msg/Bool
ros2 topic echo --once /mcl_pose
ros2 topic echo --once /obstacle_monitor_healthy std_msgs/msg/Bool
```

**Esperar:** `vision_ready: true`, MCL fresco y monitor saludable. Sólo entonces:

```bash
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
ros2 topic echo /mission/status
```

Cancelar ante cualquier pérdida de MCL, scan, visión, TF o monitor:

```bash
ros2 service call /mission/cancel std_srvs/srv/Trigger '{}'
ros2 topic pub --once "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist '{}'
```

## 10. Parada y artefactos

Al terminar cualquier prueba con movimiento:

```bash
# [Notebook]
source /tmp/tb4_lab_env.sh

ros2 service call /mission/cancel std_srvs/srv/Trigger '{}' || true
ros2 topic pub --once "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist '{}'
ros2 topic info "${CMD_VEL_TOPIC}" -v
find "${RUN_ROOT}" -maxdepth 3 -type f | sort \
  | tee "${RUN_ROOT}/artifact-manifest.txt"
```

Conservar como mínimo:

```text
acquisition/laberinto/
config/platform-parte-a-slam.yaml
config/platform-parte-a-mapa.yaml
config/platform-parte-b.yaml
config/platform-parte-c.yaml
parte_a/trajectory.json
parte_a/map.yaml
parte_a/map.pgm
logs y capturas de los gates realizados
```

No afirmar que B1–B3 o C1–C3 están aprobados hasta completar y registrar las
pruebas sobre el TurtleBot4 físico.
