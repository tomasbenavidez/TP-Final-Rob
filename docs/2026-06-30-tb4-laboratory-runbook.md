# Borrador de runbook de laboratorio TB4

**Estado:** borrador operativo inicial para Task 1.

**Alcance:** flujo real de laboratorio con TurtleBot4, desde una sesion limpia
hasta la retencion de artefactos. Este documento no cambia launch files,
paquetes, mensajes, parametros ni algoritmos.

**Fuentes:** `docs/2026-06-30-tb4-laboratory-workflow-design.md` y
`docs/2026-06-30-tb4-laboratory-workflow-implementation.md`.

## 0. Advertencias del baseline actual

- El soporte robusto de namespaces `tb4_0` y `tb4_1` todavia se implementa en
  una rama futura. En este borrador, cada comando usa `ROBOT_NS` como entrada
  de la sesion y marca las excepciones donde el codigo actual sigue teniendo
  defaults orientados a `tb4_0`.
- `ros2 run tp_slam_aruco check_bag_contract` existe, pero actualmente valida
  un contrato fijo `/tb4_0/*`. No usarlo como verificador final para `tb4_1`
  hasta que la rama de namespace soporte un argumento de namespace.
- Parte B real aun no tiene un perfil final separado. No lanzar
  `parte_b.launch.py` de simulacion directamente sobre el TB4 real.
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
export PLATFORM_CONFIG="${RUN_ROOT}/config/platform-resolved.yaml"

export ODOM_TOPIC="${ROBOT_NS}/odom"
export SCAN_TOPIC="${ROBOT_NS}/scan"
export TF_TOPIC="${ROBOT_NS}/tf"
export TF_STATIC_TOPIC="${ROBOT_NS}/tf_static"
export RGB_TOPIC="${ROBOT_NS}/oakd/rgb/preview/image_raw"
export CAMERA_INFO_TOPIC="${ROBOT_NS}/oakd/rgb/preview/camera_info"
export CMD_VEL_TOPIC="${ROBOT_NS}/cmd_vel"

mkdir -p \
  "${RUN_ROOT}/acquisition" \
  "${RUN_ROOT}/config" \
  "${RUN_ROOT}/parte_a" \
  "${RUN_ROOT}/diagnostics" \
  "${RUN_ROOT}/logs"
```

Registrar la configuracion resuelta junto con la corrida.

```bash
[Notebook laboratorio]
cat > "${PLATFORM_CONFIG}" <<EOF
run_id: ${RUN_ID}
robot_name: ${ROBOT_NAME}
robot_namespace: ${ROBOT_NS}
tb4_ssh_host: ${TB4_SSH_HOST}
odom_topic: ${ODOM_TOPIC}
scan_topic: ${SCAN_TOPIC}
tf_topic: ${TF_TOPIC}
tf_static_topic: ${TF_STATIC_TOPIC}
rgb_topic: ${RGB_TOPIC}
camera_info_topic: ${CAMERA_INFO_TOPIC}
cmd_vel_topic: ${CMD_VEL_TOPIC}
artifact_root: ${RUN_ROOT}
EOF
```

## 2. Preparar la notebook

Partir de una terminal limpia en la notebook Linux del laboratorio. No iniciar
drivers del robot desde la notebook.

```bash
[Notebook laboratorio]
cd /ruta/al/checkout/TP-Final-Rob
cd tp_final_ws
colcon build --packages-select tp_slam_interfaces tp_slam_aruco \
  tp_b_navigation tp_c_mission turtlebot3_custom_simulation
source install/setup.bash
```

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
ros2 run tf2_ros tf2_echo odom base_link
```

Si `base_link` no existe para el robot elegido, identificar el frame base real
desde TF y registrar la excepcion en `${PLATFORM_CONFIG}` antes de continuar.

```text
[RViz]
1. Fixed Frame: odom o el frame global disponible durante preflight.
2. Agregar LaserScan del topico del robot elegido.
3. Agregar Image del topico RGB elegido.
4. Agregar TF y confirmar base, LIDAR y camara.
5. No grabar si scan, imagen o TF aparecen vencidos o incoherentes.
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

Para `tb4_0` se puede correr el verificador existente como ayuda adicional,
pero no reemplaza el chequeo por `ROBOT_NS` anterior.

```bash
[Notebook laboratorio]
if [ "${ROBOT_NAME}" = "tb4_0" ]; then
  ros2 run tp_slam_aruco check_bag_contract "${BAG_DIR}"
else
  echo "check_bag_contract actual valida /tb4_0; usar solo el chequeo por ROBOT_NS."
fi
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
ros2 launch tp_slam_aruco parte_a_slam.launch.py \
  odom_topic:="${ODOM_TOPIC}" \
  bag_tf_topic:="${TF_TOPIC}" \
  bag_tf_static_topic:="${TF_STATIC_TOPIC}" \
  trajectory_file:="${TRAJECTORY_FILE}" \
  use_bag_tf:=true \
  use_sim_time:=true
```

Limitacion del baseline: el launch actual de Parte A aun fija
`image_topic` internamente para `tb4_0`. Para `tb4_1`, esta pasada requiere la
rama futura de soporte de namespace o un override equivalente antes de declarar
la corrida reproducible.

Al terminar la reproduccion del bag, cortar el launch con `Ctrl+C` para que
`graph_slam_node` escriba el JSON.

```bash
[Notebook laboratorio]
test -s "${TRAJECTORY_FILE}"
cp /tmp/aruco_detections.csv "${RUN_ROOT}/parte_a/aruco_detections.csv" 2>/dev/null || true
cp /tmp/aruco_geometry_debug.csv "${RUN_ROOT}/parte_a/aruco_geometry_debug.csv" 2>/dev/null || true
```

Gate A1/A2 antes de mapear:

```text
[RViz]
1. Revisar detecciones ArUco y debug image.
2. Revisar que los landmarks queden en posiciones razonables.
3. Revisar continuidad de trayectoria optimizada.
4. Revisar que map->odom no pegue saltos instantaneos.
5. Si los landmarks o la trayectoria son incoherentes, no generar mapa valido.
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
ros2 launch tp_slam_aruco parte_a_mapa.launch.py \
  trajectory_file:="${TRAJECTORY_FILE}" \
  odom_topic:="${ODOM_TOPIC}" \
  map_output:="${MAP_PREFIX}"
```

Limitacion del baseline: el launch actual de mapeo aun fija `scan_topic` como
`tb4_0/scan` y conserva extrinsecos LIDAR como fallback. Para `tb4_1`, no
declarar este paso robusto hasta que la rama de LIDAR por TF y namespace soporte
el topico seleccionado.

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

Gate A3:

```text
[Accion fisica]
1. No mover conos ni ArUco antes de aceptar el mapa.
2. Confirmar que el laberinto real sigue igual que durante la adquisicion.
3. Marcar el mapa y el JSON como pertenecientes al mismo RUN_ID.
```

## 11. Gates de Parte B real

Parte B real debe ejecutarse despues de aceptar mapa y landmarks. En el baseline
actual, el perfil real final queda para una rama futura; este runbook define los
gates que deberan cumplirse y evita usar el launch de simulacion sobre hardware
real.

Gate B1, localizacion sin movimiento autonomo:

```text
[RViz]
1. Cargar map.yaml y trajectory.json del mismo RUN_ID.
2. Publicar /initialpose razonable.
3. Ver nube de particulas concentrada alrededor de la pose inicial.
4. Confirmar correcciones al observar ArUco reales.
5. Confirmar que no se publica velocidad durante este gate.
```

```bash
[Notebook laboratorio]
ros2 topic echo /mcl_pose
ros2 topic echo /particlecloud
ros2 topic info "${CMD_VEL_TOPIC}" -v
```

Gate B2, navegacion basica:

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
```

Gate B3, obstaculos dinamicos:

```text
[Accion fisica]
1. Insertar obstaculo nuevo solo despues de validar B1 y B2.
2. Probar primero deteccion con robot detenido.
3. Probar replanning sin movimiento.
4. Habilitar evasion a velocidad reducida solo al final.
```

## 12. Gates de Parte C real

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

Gate C1, percepcion con robot detenido:

```bash
[Notebook laboratorio]
ros2 topic echo --once "${RGB_TOPIC}" sensor_msgs/msg/Image
ros2 topic echo --once "${CAMERA_INFO_TOPIC}" sensor_msgs/msg/CameraInfo
ros2 topic list | grep -i depth
```

```text
[RViz]
1. Confirmar RGB.
2. Identificar depth metrico alineado con RGB.
3. Confirmar TF camara -> map.
4. Ver /red_cone/debug_image, /red_cone/mask y /red_cone_pose.
5. No iniciar mision completa si depth alineado no esta validado.
```

Gate C2, aproximacion:

```text
[RViz]
1. Confirmar pose del cono en map.
2. Confirmar pose de aproximacion libre en la grilla.
3. Confirmar camino A* alcanzable.
4. Confirmar distancia final segura.
```

Gate C3, mision completa:

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

## 14. Retencion de artefactos

Antes de cerrar la sesion, conservar como minimo:

- `acquisition/laberinto/metadata.yaml` y archivos `.db3`.
- `config/platform-resolved.yaml`.
- `config/bag-info.txt`.
- `config/bag-topics.txt`.
- `parte_a/trajectory.json`.
- `parte_a/map.yaml`.
- `parte_a/map.pgm`.
- diagnosticos ArUco y geometria si existen.
- capturas o notas de RViz para gates A, B y C.
- logs de comandos relevantes.

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

## 15. Referencia separada: simulacion

La simulacion no forma parte del camino real de laboratorio de este runbook.
Usarla solamente para regresion de desarrollo o practica, siguiendo las guias
existentes bajo `docs/parte_b/` y `docs/parte_c/`.
