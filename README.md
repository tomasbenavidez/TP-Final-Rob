# TP Final Rob - Parte A

Este repo hoy esta enfocado en la **Parte A** del TP final de Robotica Autonoma:
tomar un rosbag real del TurtleBot4, hacer **Graph SLAM con ArUco**, y despues
proyectar el LIDAR sobre la trayectoria corregida para generar un mapa.

La ruta real del workspace en este repo es:

```bash
/Users/franco/TP-Final-Rob/tp_final_ws
```

## Que corre hoy

- **1ra pasada:** deteccion ArUco + Graph SLAM -> `/tmp/trayectoria.json`
- **2da pasada:** trayectoria corregida + LIDAR -> `/tmp/mapa.pgm` y `/tmp/mapa.yaml`
- **RViz:** se abre automaticamente desde `parte_a_slam.launch.py`
- **Diagnostico ArUco:** cada deteccion queda registrada en `/tmp/aruco_detections.csv`
- **Diagnostico geometrico:** las observaciones usadas por el grafo quedan en
  `/tmp/aruco_geometry_debug.csv`

Contrato principal de topicos:

- `/aruco_detections`: detecciones crudas de ArUco en frame de camara/base.
- `/aruco_base_debug`: detecciones ArUco transformadas al plano `base_link`.
- `/aruco/debug_image`: imagen de camara con tags dibujados para validar el detector.
- `/belief`: pose corregida actual del robot.
- `/poses_guardadas`: nodos/keyframes del grafo.
- `/trajectory_optimized`: trayectoria corregida completa.
- `/landmarks`: landmarks ArUco optimizados en frame `map`.
- `/landmarks_opt`: alias legado de `/landmarks` para configuraciones viejas.
- `/map`: grilla de ocupacion generada en la segunda pasada.

## Estructura minima

```text
tp_final_ws/
├── bags/
│   ├── aruco_estimation/
│   └── laberinto/
└── src/tp_slam_aruco/
    ├── config/
    ├── launch/
    └── tp_slam_aruco/
```

## Requisitos

Asumo que ya tenes ROS 2 funcionando en tu maquina y que `cv_bridge` viene de tu
instalacion de ROS.

Dependencias Python usadas por este paquete:

```bash
pip install "numpy<2" gtsam pyyaml opencv-contrib-python
python -c "import cv_bridge, cv2, gtsam, numpy; print(numpy.__version__)"
```

Si `cv_bridge` rompe con `_ARRAY_API not found`, casi seguro estas con NumPy 2.x.

## Build

```bash
cd /Users/franco/TP-Final-Rob/tp_final_ws
colcon build --packages-select tp_slam_aruco
source install/setup.bash
```

Chequeo minimo del entorno ROS:

```bash
python -c "import cv_bridge, cv2, gtsam, numpy; print(numpy.__version__)"
ros2 bag info /Users/franco/TP-Final-Rob/tp_final_ws/bags/laberinto
ros2 run tp_slam_aruco check_bag_contract /Users/franco/TP-Final-Rob/tp_final_ws/bags/laberinto
```

## Parte A - 1ra pasada

Terminal 1:

```bash
cd /Users/franco/TP-Final-Rob/tp_final_ws
source install/setup.bash
ros2 bag play /Users/franco/TP-Final-Rob/tp_final_ws/bags/laberinto --clock
```

Terminal 2:

```bash
cd /Users/franco/TP-Final-Rob/tp_final_ws
source install/setup.bash
ros2 launch tp_slam_aruco parte_a_slam.launch.py \
  calibration_file:=/Users/franco/TP-Final-Rob/tp_final_ws/src/tp_slam_aruco/config/camera_tb4_0.yaml \
  trajectory_file:=/tmp/trayectoria.json \
  use_bag_tf:=true
```

Que deberias ver:

- `/aruco_detections`
- `/aruco/debug_image`
- `/belief`
- `/poses_guardadas`
- `/landmarks`
- `/trajectory_optimized`
- TF `map -> odom`
- RViz abriendose solo, ya sincronizado con `/clock`
- La imagen de la camara con tags dibujados en el panel inferior izquierdo
- `/tmp/aruco_detections.csv` creciendo con ids, tvec, distancia, area, error
  de reproyeccion y razon de rechazo si el tag fue filtrado
- `/tmp/aruco_geometry_debug.csv` creciendo con `tf_source`, `x_base`, `y_base`,
  rango medido y residual contra el landmark optimizado disponible

Cuando frenes el launch con `Ctrl+C`, se guarda:

```bash
/tmp/trayectoria.json
```

Filtros activos por defecto en la primera pasada:

- Solo se publican detecciones ArUco con area >= `250 px`.
- Se descartan marcadores fuera de `(0.15, 3.0] m` de profundidad.
- Se descartan poses con error de reproyeccion mayor a `4 px`.
- El Graph SLAM no crea un landmark nuevo hasta verlo en `3` keyframes.
- `allowed_marker_ids` queda vacio por defecto. Si sabemos que IDs fisicos hay
  en el laberinto, conviene pasarlos como whitelist, por ejemplo:

```bash
ros2 launch tp_slam_aruco parte_a_slam.launch.py \
  calibration_file:=/Users/franco/TP-Final-Rob/tp_final_ws/src/tp_slam_aruco/config/camera_tb4_0.yaml \
  trajectory_file:=/tmp/trayectoria.json \
  use_bag_tf:=true \
  allowed_marker_ids:=4,7,11,23
```

## RViz manual (opcional)

Normalmente no hace falta porque `parte_a_slam.launch.py` ya lo abre.
Si queres levantarlo aparte, usa reloj simulado explicitamente:

```bash
cd /Users/franco/TP-Final-Rob/tp_final_ws
source install/setup.bash
ros2 run rviz2 rviz2 \
  --ros-args -p use_sim_time:=true \
  -d /Users/franco/TP-Final-Rob/tp_final_ws/src/tp_slam_aruco/config/rviz_config.rviz
```

## Parte A - 2da pasada

Terminal 1:

```bash
cd /Users/franco/TP-Final-Rob/tp_final_ws
source install/setup.bash
ros2 bag play /Users/franco/TP-Final-Rob/tp_final_ws/bags/laberinto --clock
```

Terminal 2:

```bash
cd /Users/franco/TP-Final-Rob/tp_final_ws
source install/setup.bash
ros2 launch tp_slam_aruco parte_a_mapa.launch.py \
  trajectory_file:=/tmp/trayectoria.json \
  map_output:=/tmp/mapa
```

Salida esperada al cortar con `Ctrl+C`:

```bash
/tmp/mapa.pgm
/tmp/mapa.yaml
```

## Verificacion recomendada

### Bag corto de ArUco

Usalo antes del bag largo para validar calibracion, TF y detecciones:

```bash
cd /Users/franco/TP-Final-Rob/tp_final_ws
source install/setup.bash
ros2 bag play /Users/franco/TP-Final-Rob/tp_final_ws/bags/aruco_estimation --clock
```

En otra terminal:

```bash
cd /Users/franco/TP-Final-Rob/tp_final_ws
source install/setup.bash
ros2 launch tp_slam_aruco parte_a_slam.launch.py \
  calibration_file:=/Users/franco/TP-Final-Rob/tp_final_ws/src/tp_slam_aruco/config/camera_tb4_0.yaml \
  trajectory_file:=/tmp/trayectoria_aruco_estimation.json \
  use_bag_tf:=true
```

Chequeos utiles:

```bash
ros2 topic echo /aruco_detections --once
ros2 topic echo /aruco_base_debug --once
ros2 topic echo /landmarks --once
ros2 run tf2_ros tf2_echo base_link oakd_rgb_camera_optical_frame
python - <<'PY'
import csv
from collections import Counter
rows=list(csv.DictReader(open('/tmp/aruco_detections.csv')))
accepted=[r for r in rows if r['accepted'] == '1']
print('raw rows:', len(rows), 'accepted:', len(accepted))
print('accepted ids:', sorted(Counter(r['id'] for r in accepted).items()))
print('rejections:', Counter(r['reason'] for r in rows if r['accepted'] == '0'))
PY
```

Para separar problema de geometria vs Graph SLAM al terminar la primera pasada:

```bash
python - <<'PY'
import csv, statistics
rows=[r for r in csv.DictReader(open('/tmp/aruco_geometry_debug.csv')) if r['residual_range']]
res=[abs(float(r['residual_range'])) for r in rows]
print('rows with residual:', len(rows))
print('fallback rows:', sum(1 for r in rows if r['tf_source'] == 'fallback'))
if res:
    print('abs residual min/median/max:', min(res), statistics.median(res), max(res))
    print('large residual rows:', sum(1 for v in res if v > 0.4))
PY
```

### Tests

Con un entorno ROS completo:

```bash
cd /Users/franco/TP-Final-Rob/tp_final_ws
source install/setup.bash
python -m pytest src/tp_slam_aruco/test -q
```

El smoke test con rosbag esta apagado por defecto para no correr bags grandes
en cada test:

```bash
RUN_ROS_SMOKE=1 python -m pytest src/tp_slam_aruco/test/test_ros_smoke.py -q
```

## Troubleshooting corto

### No anda TF o RViz queda vacio

Verifica estas dos cosas primero:

```bash
ros2 bag play /Users/franco/TP-Final-Rob/tp_final_ws/bags/laberinto --clock
ros2 run tf2_ros tf2_echo base_link oakd_rgb_camera_optical_frame
```

La 1ra pasada ahora levanta un **TF bridge** para republicar el TF del bag
(``/tb4_0/tf`` y ``/tb4_0/tf_static``) en los topicos estandar (`/tf`,
`/tf_static`).

### La geometria sigue mal

- Deja `use_bag_tf:=true`
- Los extrinsecos numericos quedaron solo como fallback
- El fallback actual del TB4 grabado en este bag es:

```bash
camera_tx:=-0.0596 camera_ty:=0.0 camera_yaw:=0.0
```

### No aparece `trajectory.json`

- Asegurate de cortar el launch con `Ctrl+C`
- Usa una ruta escribible, por ejemplo `/tmp/trayectoria.json`

### `cv_bridge` rompe al arrancar

```bash
pip install "numpy<2"
python -c "import cv_bridge; print('cv_bridge ok')"
```

## Alcance actual

Este repo esta preparado para correr y depurar **Parte A**.

- **Parte B** y **Parte C** siguen siendo contexto del TP, pero no estan
  empaquetadas aca como flujos end-to-end con el mismo nivel de soporte.
