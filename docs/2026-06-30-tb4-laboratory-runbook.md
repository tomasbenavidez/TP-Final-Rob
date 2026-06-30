# Runbook final de laboratorio TB4

**Estado:** procedimiento final ejecutable. La preparación local y con bags
está verificada; B1–B3 y C1–C3 requieren un TurtleBot4 físico y permanecen
pendientes hasta registrar evidencia en el laboratorio.

**Alcance:** flujo real de laboratorio con TurtleBot4, desde una sesion limpia
hasta la retencion de artefactos. Los launch files aceptan `robot_namespace`
para seleccionar `tb4_0` o `tb4_1` una vez por corrida.

**Fuentes:** `docs/2026-06-30-tb4-laboratory-workflow-design.md` y
`docs/2026-06-30-tb4-laboratory-workflow-implementation.md`.

## 0. Contratos de ejecucion

- El soporte de namespaces `tb4_0` y `tb4_1` se selecciona con
  `robot_namespace`. `tb4_0` sigue siendo el default de compatibilidad.
- `ros2 run tp_a_slam_aruco check_bag_contract` acepta
  `--robot-namespace tb4_0|tb4_1` y valida el contrato del robot elegido.
- Parte B real se lanza con `profile:=real_tb4`; este perfil no inicia el sensor
  virtual de landmarks y activa los safety gates.
- Parte C real usa mapa y landmarks del mismo `RUN_ID`, y un topico depth
  alineado con RGB validado antes de iniciar la mision.
- `state_machine` es el unico productor normal de velocidad. Un operador puede
  publicar un `Twist` cero como parada manual.
- Este documento evita comandos de Gazebo en la ruta principal. La simulacion
  queda fuera del procedimiento real de laboratorio.

## 1. Variables de sesion

Elegir el robot una sola vez y derivar todo desde esa seleccion. Usar una ruta
de artefactos fuera del checkout para no escribir resultados bajo `share/` ni
mezclarlos con archivos versionados.

```bash
[Notebook laboratorio]
export ROBOT_NAME=tb4_0              # cambiar a tb4_1 si corresponde
export ROBOT_NS="/${ROBOT_NAME}"
export TB4_SSH_HOST="${ROBOT_NAME}"  # ajustar si el hostname SSH difiere
export RUN_ID="$(date +%Y%m%d-%H%M)-${ROBOT_NAME}"
export RUN_ROOT="${HOME}/tb4_laboratorio_runs/${RUN_ID}"

export BAG_DIR="${RUN_ROOT}/acquisition/laberinto"
export TRAJECTORY_FILE="${RUN_ROOT}/parte_a/trajectory.json"
export MAP_PREFIX="${RUN_ROOT}/parte_a/map"
export MAP_YAML="${MAP_PREFIX}.yaml"
export MAP_PGM="${MAP_PREFIX}.pgm"
export PLATFORM_A_SLAM="${RUN_ROOT}/config/platform-parte-a-slam.yaml"
export PLATFORM_A_MAPA="${RUN_ROOT}/config/platform-parte-a-mapa.yaml"
export PLATFORM_B="${RUN_ROOT}/config/platform-parte-b.yaml"
export PLATFORM_C="${RUN_ROOT}/config/platform-parte-c.yaml"

export ODOM_TOPIC="${ROBOT_NS}/odom"
export SCAN_TOPIC="${ROBOT_NS}/scan"
export TF_TOPIC="${ROBOT_NS}/tf"
export TF_STATIC_TOPIC="${ROBOT_NS}/tf_static"
export RGB_TOPIC="${ROBOT_NS}/oakd/rgb/preview/image_raw"
export CAMERA_INFO_TOPIC="${ROBOT_NS}/oakd/rgb/preview/camera_info"
export DEPTH_TOPIC=""  # completar en C1 con depth metrico alineado con RGB
export CMD_VEL_TOPIC="${ROBOT_NS}/cmd_vel"

mkdir -p \
  "${RUN_ROOT}/acquisition" \
  "${RUN_ROOT}/config" \
  "${RUN_ROOT}/parte_a" \
  "${RUN_ROOT}/diagnostics" \
  "${RUN_ROOT}/logs"
```

Cada launch escribe atómicamente su configuración resuelta en
`config/platform-<etapa>.yaml`. Pasar siempre `run_id:="${RUN_ID}"`; no crear
ni reutilizar manualmente un único `platform-resolved.yaml`.

## 2. Preparar la notebook y las terminales

Partir de una terminal limpia en la notebook Linux del laboratorio. No iniciar
drivers del robot desde la notebook.

```bash
[Notebook laboratorio]
cd /ruta/al/checkout/TP-Final-Rob
source "${HOME}/miniforge3/etc/profile.d/conda.sh"
conda activate rosenv_mf

# Limpiar sólo productos de los paquetes renombrados, nunca bags ni runs:
rm -rf \
  tp_final_ws/build/tp_slam_interfaces \
  tp_final_ws/build/tp_slam_aruco \
  tp_final_ws/install/tp_slam_interfaces \
  tp_final_ws/install/tp_slam_aruco

cd tp_final_ws
colcon build --packages-select tp_platform tp_interfaces tp_a_slam_aruco \
  tp_b_navigation tp_c_mission turtlebot3_custom_simulation
source install/setup.bash
```

Repetir `conda activate rosenv_mf` y `source install/setup.bash` en cada
terminal. Layout recomendado:

| Terminal | Uso |
|---|---|
| N1 | discovery, topicos y parada manual |
| N2 | bag play o launch principal |
| N3 | RViz y observacion de estados |
| N4 | SSH al TB4 durante adquisicion |

No mezclar una terminal con `use_sim_time` residual y una corrida real.

Confirmar que la notebook usa el dominio y RMW esperados por el laboratorio.
Los valores concretos dependen de la red del dia.

```bash
[Notebook laboratorio]
printenv ROS_DOMAIN_ID
printenv RMW_IMPLEMENTATION
ros2 node list
```

## 3. Seleccionar robot y comprobar red

Confirmar fisicamente que el robot elegido coincide con `ROBOT_NAME`.

```text
[Accion fisica]
1. Identificar el TB4 disponible en el laboratorio.
2. Confirmar si su namespace operativo es tb4_0 o tb4_1.
3. Confirmar que el e-stop o metodo de parada inmediata funciona.
4. Designar una persona responsable de detener el robot.
```

Comprobar que se ven topicos del robot elegido y que no hay ambiguedad con el
otro TB4.

```bash
[Notebook laboratorio]
ros2 topic list | sort | tee "${RUN_ROOT}/config/live-topics-before-bag.txt"
grep "^${ROBOT_NS}/" "${RUN_ROOT}/config/live-topics-before-bag.txt"
grep -E "^/(tb4_0|tb4_1)/" "${RUN_ROOT}/config/live-topics-before-bag.txt"
```

Si aparecen topicos criticos de ambos robots, detener el procedimiento y
corregir el entorno ROS 2 antes de mover el robot.

## 4. Seguridad antes de mover

La seguridad se valida antes de teleoperar, grabar bags o lanzar cualquier
autonomia. Durante mapeo manual deben estar apagados `state_machine`, misiones
y cualquier otro productor normal de velocidad.

```bash
[Notebook laboratorio]
ros2 topic info "${CMD_VEL_TOPIC}" -v
ros2 node list | grep -E "state_machine|mission_manager|graph_slam|mcl_localization" || true
ros2 topic pub --once "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist '{}'
```

```text
[Accion fisica]
1. Dejar el robot en una zona corta y libre.
2. Probar teleoperacion con velocidad reducida.
3. Soltar el control y confirmar que el robot queda detenido.
4. Probar el metodo de parada inmediata.
5. No entrar al laberinto hasta que la parada este verificada.
```

## 5. Preflight de sensores y TF

Validar mensajes frescos de odometria, LIDAR, RGB y `CameraInfo`.

```bash
[Notebook laboratorio]
ros2 topic echo --once "${ODOM_TOPIC}" nav_msgs/msg/Odometry
ros2 topic echo --once "${SCAN_TOPIC}" sensor_msgs/msg/LaserScan
ros2 topic echo --once "${CAMERA_INFO_TOPIC}" sensor_msgs/msg/CameraInfo
ros2 topic hz "${ODOM_TOPIC}"
ros2 topic hz "${SCAN_TOPIC}"
ros2 topic hz "${RGB_TOPIC}"
```

Validar TF usando los frames publicados por los mensajes. No asumir que los
frames se construyen concatenando el namespace.

```bash
[Notebook laboratorio]
ros2 topic echo --once "${SCAN_TOPIC}" sensor_msgs/msg/LaserScan > "${RUN_ROOT}/config/scan-sample.txt"
ros2 topic echo --once "${RGB_TOPIC}" sensor_msgs/msg/Image > "${RUN_ROOT}/config/rgb-sample.txt"
SCAN_FRAME="$(sed -n 's/^[[:space:]]*frame_id: //p' "${RUN_ROOT}/config/scan-sample.txt" | head -1)"
RGB_FRAME="$(sed -n 's/^[[:space:]]*frame_id: //p' "${RUN_ROOT}/config/rgb-sample.txt" | head -1)"
ros2 run tf2_ros tf2_echo odom "${ROBOT_NAME}/base_link"
# Si los frames no llevan prefijo, usar exactamente el frame publicado:
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo "${ROBOT_NAME}/base_link" "${SCAN_FRAME}"
ros2 run tf2_ros tf2_echo "${ROBOT_NAME}/base_link" "${RGB_FRAME}"
```

Si `base_link` no existe para el robot elegido, identificar el frame base real
desde TF y registrarlo en las notas de la corrida antes de continuar. No
anteponer el namespace a un frame que el propio mensaje publica sin él.

```text
[RViz]
1. Fixed Frame: odom o el frame global disponible durante preflight.
2. Agregar LaserScan del topico del robot elegido.
3. Agregar Image del topico RGB elegido.
4. Agregar TF y confirmar base, LIDAR y camara.
5. Repetir la comprobación seleccionando `tb4_0` o `tb4_1`, según
   `ROBOT_NAME`, y verificar que RViz no consume tópicos del otro robot.
6. No grabar si scan, imagen o TF aparecen vencidos o incoherentes.
```

## 6. Grabar rosbag onboard

La adquisicion de Parte A se graba en el TB4 para no depender de que la imagen
de alta tasa llegue a la notebook. La teleoperacion puede originarse en la
notebook, pero el bag queda almacenado localmente en el robot.

Crear una carpeta de corrida en el robot.

```bash
[TB4 por SSH]
export ROBOT_NAME=tb4_0              # usar el mismo valor elegido en notebook
export ROBOT_NS="/${ROBOT_NAME}"
export RUN_ID=YYYYMMDD-HHMM-${ROBOT_NAME}
export TB4_RUN_ROOT="${HOME}/tb4_laboratorio_runs/${RUN_ID}"
mkdir -p "${TB4_RUN_ROOT}/acquisition"
```

Grabar el contrato minimo y topicos recomendados disponibles. Si algun topico
opcional no existe, quitarlo del comando y registrarlo en notas de la corrida.

```bash
[TB4 por SSH]
ros2 bag record \
  "${ROBOT_NS}/odom" \
  "${ROBOT_NS}/scan" \
  "${ROBOT_NS}/oakd/rgb/preview/image_raw" \
  "${ROBOT_NS}/oakd/rgb/preview/camera_info" \
  "${ROBOT_NS}/tf" \
  "${ROBOT_NS}/tf_static" \
  "${ROBOT_NS}/imu" \
  -o "${TB4_RUN_ROOT}/acquisition/laberinto"
```

Recorrer manualmente el laberinto una sola vez.

```text
[Accion fisica]
1. Iniciar grabacion antes de mover.
2. Teleoperar con velocidad reducida.
3. Cubrir el laberinto buscando reobservaciones de ArUco.
4. Evitar giros bruscos innecesarios frente a paredes finas.
5. Detener el robot antes de cortar la grabacion.
6. Cortar el bag con Ctrl+C y esperar a que cierre metadata.yaml.
```

Validar en el TB4 que el bag cerro correctamente.

```bash
[TB4 por SSH]
test -f "${TB4_RUN_ROOT}/acquisition/laberinto/metadata.yaml"
ros2 bag info "${TB4_RUN_ROOT}/acquisition/laberinto"
du -sh "${TB4_RUN_ROOT}/acquisition/laberinto"
```

## 7. Copiar bag a la notebook

Copiar la carpeta completa del bag, no solo el `.db3`.

```bash
[Notebook laboratorio]
mkdir -p "${RUN_ROOT}/acquisition"
rsync -av --progress \
  "${TB4_SSH_HOST}:tb4_laboratorio_runs/${RUN_ID}/acquisition/laberinto" \
  "${RUN_ROOT}/acquisition/"
```

Validar que la copia abre localmente.

```bash
[Notebook laboratorio]
ros2 bag info "${BAG_DIR}" | tee "${RUN_ROOT}/config/bag-info.txt"
cp "${BAG_DIR}/metadata.yaml" "${RUN_ROOT}/config/bag-metadata.yaml"
grep "name:" "${BAG_DIR}/metadata.yaml" | tee "${RUN_ROOT}/config/bag-topics.txt"
```

Validar manualmente el contrato minimo para el namespace elegido.

```bash
[Notebook laboratorio]
for topic in \
  "${ODOM_TOPIC}" \
  "${SCAN_TOPIC}" \
  "${RGB_TOPIC}" \
  "${CAMERA_INFO_TOPIC}" \
  "${TF_TOPIC}" \
  "${TF_STATIC_TOPIC}"
do
  grep -q "name: ${topic}$" "${BAG_DIR}/metadata.yaml" \
    && echo "OK ${topic}" \
    || echo "FALTA ${topic}"
done
```

El verificador de Parte A debe ejecutarse con el mismo namespace elegido para
la corrida.

```bash
[Notebook laboratorio]
ros2 run tp_a_slam_aruco check_bag_contract "${BAG_DIR}" \
  --robot-namespace "${ROBOT_NAME}"
```

Reproducir una muestra corta antes de procesar.

```bash
[Notebook laboratorio]
ros2 bag play "${BAG_DIR}" --clock --start-offset 0 --duration 10
```

```text
[RViz]
1. Confirmar que se ven scan, imagen y TF durante la reproduccion corta.
2. Confirmar que los timestamps avanzan con /clock.
3. No iniciar Parte A si el bag no reproduce datos criticos.
```

## 8. Parte A, pasada 1: ArUco y Graph SLAM

La primera pasada consume el bag offline con `--clock` y guarda trayectoria y
landmarks optimizados. No se ejecuta Parte B al mismo tiempo.

Terminal 1:

```bash
[Notebook laboratorio]
cd /ruta/al/checkout/TP-Final-Rob/tp_final_ws
source install/setup.bash
ros2 bag play "${BAG_DIR}" --clock
```

Terminal 2:

```bash
[Notebook laboratorio]
cd /ruta/al/checkout/TP-Final-Rob/tp_final_ws
source install/setup.bash
ros2 launch tp_a_slam_aruco parte_a_slam.launch.py \
  robot_namespace:="${ROBOT_NAME}" \
  artifact_dir:="${RUN_ROOT}" \
  run_id:="${RUN_ID}" \
  odom_topic:="${ODOM_TOPIC}" \
  bag_tf_topic:="${TF_TOPIC}" \
  bag_tf_static_topic:="${TF_STATIC_TOPIC}" \
  trajectory_file:="${TRAJECTORY_FILE}" \
  use_bag_tf:=true \
  use_sim_time:=true
```

Al terminar la reproduccion del bag, cortar el launch con `Ctrl+C` para que
`graph_slam_node` escriba el JSON.

```bash
[Notebook laboratorio]
test -s "${TRAJECTORY_FILE}"
cp /tmp/aruco_detections.csv "${RUN_ROOT}/parte_a/aruco_detections.csv" 2>/dev/null || true
cp /tmp/aruco_geometry_debug.csv "${RUN_ROOT}/parte_a/aruco_geometry_debug.csv" 2>/dev/null || true
```

Checkpoints A1 y A2 antes de mapear:

```text
[A1 - deteccion y geometria]
APROBAR: IDs esperados, detecciones estables, reproyeccion aceptable y TF
camara->base disponible o fallback documentado.
RECHAZAR: IDs espurios persistentes, detecciones sin TF o geometria incoherente.

[A2 - Graph SLAM]
APROBAR: trajectory.json no vacio, landmarks razonables, trayectoria continua y
map->odom sin saltos instantaneos.
RECHAZAR: JSON ausente, landmarks divergentes o trayectoria discontinua.
```

## 9. Parte A, pasada 2: grilla de ocupacion

La segunda pasada usa el mismo bag y el JSON de la primera pasada. No repetir
fisicamente la trayectoria.

Terminal 1:

```bash
[Notebook laboratorio]
cd /ruta/al/checkout/TP-Final-Rob/tp_final_ws
source install/setup.bash
ros2 bag play "${BAG_DIR}" --clock
```

Terminal 2:

```bash
[Notebook laboratorio]
cd /ruta/al/checkout/TP-Final-Rob/tp_final_ws
source install/setup.bash
ros2 launch tp_a_slam_aruco parte_a_mapa.launch.py \
  robot_namespace:="${ROBOT_NAME}" \
  artifact_dir:="${RUN_ROOT}" \
  run_id:="${RUN_ID}" \
  trajectory_file:="${TRAJECTORY_FILE}" \
  odom_topic:="${ODOM_TOPIC}" \
  scan_topic:="${SCAN_TOPIC}" \
  map_output:="${MAP_PREFIX}"
```

Al terminar la reproduccion, cortar el launch con `Ctrl+C` para exportar mapa.

```bash
[Notebook laboratorio]
test -s "${MAP_YAML}"
test -s "${MAP_PGM}"
```

## 10. Inspeccionar y aceptar el mapa

Cargar el mapa generado sin copiarlo a `share/` ni reemplazar mapas versionados.

Terminal 1:

```bash
[Notebook laboratorio]
source /ruta/al/checkout/TP-Final-Rob/tp_final_ws/install/setup.bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="${MAP_YAML}"
```

Terminal 2:

```bash
[Notebook laboratorio]
source /ruta/al/checkout/TP-Final-Rob/tp_final_ws/install/setup.bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

Terminal 3:

```bash
[Notebook laboratorio]
source /ruta/al/checkout/TP-Final-Rob/tp_final_ws/install/setup.bash
rviz2
```

```text
[RViz]
1. Fixed Frame: map.
2. Agregar Map en /map con Durability Policy Transient Local si hace falta.
3. Verificar paredes completas, aperturas transitables y ausencia de paredes dobles severas.
4. Verificar que no haya conexiones falsas a traves de paredes.
5. Rechazar el mapa si no es navegable para el TB4.
```

Checkpoint A3:

```text
[Accion fisica]
1. No mover conos ni ArUco antes de aceptar el mapa.
2. Confirmar que el laberinto real sigue igual que durante la adquisicion.
3. Marcar el mapa y el JSON como pertenecientes al mismo RUN_ID.

APROBAR: map.yaml/PGM cargan, paredes y aperturas coinciden con el escenario,
no hay conexiones falsas y el JSON pertenece al mismo RUN_ID.
RECHAZAR: paredes dobles severas, aperturas cerradas, mapa no navegable o
artefactos mezclados entre corridas.
```

## 11. Parte B real

Parte B real se ejecuta despues de aceptar mapa y landmarks. Antes de lanzar,
dejar el robot detenido, probar la parada manual y mantener el e-stop accesible.
El estado de la evidencia local previa está en
`docs/parte_b/tb4-mcl-obstacle-diagnostic.md`.

Terminal N1, parada disponible antes de cualquier goal:

```bash
[Notebook laboratorio]
ros2 topic pub --once "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist '{}'
ros2 topic info "${CMD_VEL_TOPIC}" -v
```

Terminal N2:

```bash
[Notebook laboratorio]
cd /ruta/al/checkout/TP-Final-Rob/tp_final_ws
source "${HOME}/miniforge3/etc/profile.d/conda.sh"
conda activate rosenv_mf
source install/setup.bash
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

Checkpoint B1, localizacion sin movimiento autonomo:

**Estado inicial: PENDIENTE EN HARDWARE.** Ejecutar B1 con
`cmd_vel_topic:=/test/cmd_vel` y confirmar que ningún publisher aparece en
`${CMD_VEL_TOPIC}` ni el robot se mueve. Volver al tópico real únicamente para
B2, después de aprobar B1 y repetir la parada segura: detener el launch y
relanzar el mismo comando reemplazando
`cmd_vel_topic:=/test/cmd_vel` por `cmd_vel_topic:="${CMD_VEL_TOPIC}"`.

```text
[RViz]
1. Cargar map.yaml y trajectory.json del mismo RUN_ID.
2. Publicar /initialpose razonable.
3. Ver nube de particulas concentrada alrededor de la pose inicial.
4. Confirmar correcciones al observar ArUco reales.
5. Confirmar que no se publica velocidad durante este gate.
6. Confirmar `/obstacle_monitor_healthy: true`.

APROBAR: MCL fresco y concentrado, covarianza dentro de limites, scan/monitor
frescos, TF map->base disponible y robot inmovil.
RECHAZAR: pose vencida, covarianza alta, scan/monitor stale, TF ausente o
cualquier velocidad inesperada.
```

```bash
[Notebook laboratorio]
ros2 topic echo /mcl_pose
ros2 topic echo /particlecloud
ros2 topic echo --once /obstacle_monitor_healthy std_msgs/msg/Bool
ros2 topic info "${CMD_VEL_TOPIC}" -v
```

Checkpoint B2, navegacion basica:

**Estado inicial: PENDIENTE EN HARDWARE.** Antes de cada goal: cancelar toda
misión previa, publicar `Twist` cero, confirmar robot detenido y volver a
probar el método de parada inmediata. Hacer primero un goal corto y luego
múltiples goals sólo si el primero es seguro.

```text
[Accion fisica]
1. Mantener persona responsable junto al e-stop.
2. Usar velocidad reducida.
3. Elegir primero un objetivo cercano, libre y despejado.
4. Avanzar a objetivos mas lejanos solo si el robot mantiene clearance.
```

```text
[RViz]
1. Enviar un goal cercano.
2. Ver /plan antes de permitir movimiento.
3. Ver seguimiento, orientacion final y error de llegada.
4. Cancelar si la pose MCL se vuelve incoherente con scan y mapa.

APROBAR: plan valido antes de mover, seguimiento con clearance y llegada dentro
de tolerancia con orientacion final.
RECHAZAR: cruce de paredes, movimiento sin plan, oscilacion insegura o perdida
de localizacion/TF/scan/monitor.
```

Checkpoint B3, obstaculos dinamicos:

**Estado inicial: PENDIENTE EN HARDWARE.** Antes de cada subprueba: cancelar,
publicar cero y confirmar robot detenido. El orden obligatorio es detección
detenido, replan sin movimiento y recién después evasión a velocidad reducida
con recuperación de localización.

```text
[Accion fisica]
1. Insertar obstaculo nuevo solo despues de validar B1 y B2.
2. Probar primero deteccion con robot detenido.
3. Probar replanning sin movimiento.
4. Habilitar evasion a velocidad reducida solo al final.

APROBAR: obstaculo no mapeado aparece en `/dynamic_obstacles`, dispara
`/obstacle_detected` y produce evasion/replan sin colision.
RECHAZAR: pared estatica tratada repetidamente como obstaculo nuevo, obstaculo
ignorado o movimiento cuando el monitor deja de estar saludable.
```

## 12. Parte C real

Parte C real usa el mismo mapa y landmarks aceptados. Los conos se agregan
despues del mapeo; los ArUco y paredes no se mueven.

Preparar escenario:

```text
[Accion fisica]
1. Mantener paredes y ArUco en las posiciones usadas para Parte A/B.
2. Agregar el cono rojo despues de aceptar el mapa.
3. Agregar distractores de otros colores.
4. Evitar bloquear todas las rutas al objetivo.
```

Checkpoint C1, percepcion con robot detenido:

**Estado inicial: PENDIENTE EN HARDWARE.** Cancelar, publicar cero, confirmar
robot detenido y mantener la salida de velocidad física sin publishers durante
esta verificación de percepción.

```bash
[Notebook laboratorio]
ros2 topic echo --once "${RGB_TOPIC}" sensor_msgs/msg/Image
ros2 topic echo --once "${CAMERA_INFO_TOPIC}" sensor_msgs/msg/CameraInfo
ros2 topic list | grep -i depth
```

Elegir el topico que entregue depth metrico alineado con el RGB y conservarlo:

```bash
[Notebook laboratorio]
export DEPTH_TOPIC="${ROBOT_NS}/RUTA/DEPTH_ALINEADO"
ros2 topic echo --once "${DEPTH_TOPIC}" sensor_msgs/msg/Image
ros2 topic info "${RGB_TOPIC}" -v
ros2 topic info "${DEPTH_TOPIC}" -v
```

```text
[RViz]
1. Confirmar RGB.
2. Identificar depth metrico alineado con RGB.
3. Confirmar que RGB y depth tienen las mismas dimensiones y corresponden al
   mismo campo visual; registrar encoding, ancho, alto y timestamps.
4. Confirmar TF camara -> map en el timestamp de RGB.
5. Ver /red_cone/debug_image, /red_cone/mask y /red_cone_pose.
6. Confirmar `/vision_ready: true`; ante cualquier incumplimiento debe ser
   `false`.
7. No iniciar mision completa si depth alineado no esta validado.

APROBAR: cono rojo estable produce pose en map; distractores no rojos no
producen deteccion; depth, CameraInfo y TF son coherentes.
RECHAZAR: depth no alineado/no metrico, pose fuera del piso/mapa, deteccion de
distractores visuales o vision vencida.
```

Antes de lanzar Parte C, cancelar cualquier mision anterior y publicar cero:

```bash
[Notebook laboratorio]
ros2 service call /mission/cancel std_srvs/srv/Trigger '{}' || true
ros2 topic pub --once "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist '{}'
```

Detener el launch de Parte B y arrancar Parte C en N2:

```bash
[Notebook laboratorio]
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

Checkpoint C2, aproximacion:

**Estado inicial: PENDIENTE EN HARDWARE.** Cancelar, publicar cero y confirmar
robot detenido antes de validar la pose; esta prueba no autoriza movimiento.

```text
[RViz]
1. Confirmar pose del cono en map.
2. Confirmar pose de aproximacion libre en la grilla.
3. Confirmar camino A* alcanzable.
4. Confirmar distancia final segura.

APROBAR: pose de aproximacion libre, A* alcanzable y standoff seguro.
RECHAZAR: objetivo dentro de pared/capa dinamica, plan ausente o visibilidad de
camara usada como sustituto de la grilla.
```

Checkpoint C3, mision completa:

**Estado inicial: PENDIENTE EN HARDWARE.** Sólo habilitarla después de aprobar
C1 y C2. Antes de cada intento: cancelar, publicar cero, confirmar parada
manual/e-stop y probar luego cancelación, datos stale y parada de emergencia.

```bash
[Notebook laboratorio]
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
ros2 topic echo /mission/status
```

```text
[Accion fisica]
1. Mantener parada manual disponible.
2. Cancelar si se pierde localizacion, scan, vision o TF.
3. Confirmar que distractores no rojos se ignoran.
4. Confirmar que la mision publica FOUND solo para el cono rojo.

APROBAR: exploracion progresa, el cono rojo confirmado termina en FOUND, los
distractores no rojos se ignoran visualmente y los objetos fisicos siguen
siendo tratados por LIDAR.
RECHAZAR: movimiento con MCL/vision/scan/monitor/TF vencido, colision, cruce de
pared o FOUND para un distractor.
```

Cancelacion normal:

```bash
[Notebook laboratorio]
ros2 service call /mission/cancel std_srvs/srv/Trigger '{}'
```

## 13. Apagado y parada segura

Al terminar cualquier etapa con movimiento, cancelar mision, detener el
controlador y verificar velocidad cero.

```bash
[Notebook laboratorio]
ros2 service call /mission/cancel std_srvs/srv/Trigger '{}' || true
ros2 topic pub --once "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist '{}'
ros2 topic echo --once "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist
```

```text
[Accion fisica]
1. Confirmar robot detenido.
2. Alejar obstaculos agregados si ya no forman parte de la prueba.
3. Apagar o dejar cargando el TB4 segun procedimiento del laboratorio.
4. No desmontar el escenario antes de copiar artefactos criticos.
```

## 14. Troubleshooting

| Sintoma | Comprobacion y accion |
|---|---|
| No hay discovery | Igualar `ROS_DOMAIN_ID` y `RMW_IMPLEMENTATION`; comprobar red, reiniciar `ros2 daemon`, volver a listar nodos. |
| Topicos del TB4 incorrecto | Revisar `ROBOT_NAME`, `ROBOT_NS` y el `platform-<etapa>.yaml`; no usar overrides de otro namespace. |
| Falta TF | Inspeccionar frame de scan/camara y `ros2 run tf2_ros tf2_echo`; no inventar frames con el namespace. |
| Depth no sirve | Confirmar encoding, escala metrica, alineacion con RGB y timestamps; repetir C1. |
| MCL no converge | Revisar `/initialpose`, mapa/landmarks del mismo RUN_ID, observaciones ArUco y covarianza. |
| `scan_stale` | Verificar frecuencia, reloj real (`use_sim_time:=false`) y namespace del scan. |
| Monitor stale/unhealthy | Revisar `/obstacle_monitor_healthy`, `/map`, MCL, scan y TF del sensor; no continuar hasta `true`. |
| Plan fallido | Verificar goal libre, mapa aceptado y capa `/dynamic_obstacles`; cancelar y elegir objetivo seguro. |
| Vision stale | Revisar RGB, CameraInfo, depth y TF; `mission_manager` debe cancelar y publicar `FAILED`. |

No desactivar safety gates para hacer desaparecer un diagnostico.

## 15. Retencion de artefactos

Antes de cerrar la sesion, conservar como minimo:

- `acquisition/laberinto/metadata.yaml` y archivos `.db3`.
- `config/platform-parte-a-slam.yaml`.
- `config/platform-parte-a-mapa.yaml`.
- `config/platform-parte-b.yaml`.
- `config/platform-parte-c.yaml`.
- `config/bag-info.txt`.
- `config/bag-topics.txt`.
- `parte_a/trajectory.json`.
- landmarks optimizados incluidos en `parte_a/trajectory.json`.
- `parte_a/map.yaml`.
- `parte_a/map.pgm`.
- diagnosticos ArUco y geometria si existen.
- capturas o notas de RViz para gates A, B y C.
- logs de comandos relevantes.
- resultado aprobado/rechazado de A1-A3, B1-B3 y C1-C3.

Estado de referencia antes del laboratorio:

| Gate | Estado | Evidencia requerida |
|---|---|---|
| A1–A3, bag conocido | Verificado localmente con salvedades documentadas | `docs/parte_a/tb4-map-comparison.md` |
| B1–B3 | Pendiente en hardware | MCL, comandos, planes, obstáculos y parada segura |
| C1–C3 | Pendiente en hardware | RGB-D alineado, pose de aproximación, misión y fallos seguros |

Crear un resumen textual de la corrida.

```bash
[Notebook laboratorio]
cat > "${RUN_ROOT}/README-run.txt" <<EOF
RUN_ID=${RUN_ID}
ROBOT_NAME=${ROBOT_NAME}
ROBOT_NS=${ROBOT_NS}
BAG_DIR=${BAG_DIR}
TRAJECTORY_FILE=${TRAJECTORY_FILE}
MAP_YAML=${MAP_YAML}
MAP_PGM=${MAP_PGM}

Resultado Parte A:
Resultado Parte B real:
Resultado Parte C real:
Incidentes de seguridad:
Notas:
EOF

find "${RUN_ROOT}" -maxdepth 3 -type f | sort > "${RUN_ROOT}/artifact-manifest.txt"
```

## 16. Referencia separada: simulacion

La simulacion no forma parte del camino real de laboratorio de este runbook.
Usarla solamente para regresion de desarrollo o practica, siguiendo las guias
existentes bajo `docs/parte_b/` y `docs/parte_c/`.
