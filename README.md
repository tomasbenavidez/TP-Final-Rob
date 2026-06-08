# tp_slam_aruco — TP Final Robótica (Opción 3)

Graph SLAM sobre datos reales del TurtleBot4 usando marcadores ArUco y LIDAR.
Implementado como paquete ROS 2 (ament_python).

---

## Estructura del paquete

```
tp_slam_aruco/
├── config/
│   ├── camera_tb4_0.yaml          # Calibración intrínseca de la cámara OAK-D (TB4 #0)
│   └── rviz_config.rviz           # Configuración de RViz2
├── launch/
│   ├── parte_a_slam.launch.py     # 1ª pasada: Graph SLAM + ArUco → trayectoria.json
│   └── parte_a_mapa.launch.py     # 2ª pasada: trayectoria.json + LIDAR → mapa.pgm/.yaml
├── runs/
│   └── trayectoria.json           # Salida de la 1ª pasada (poses + timestamps + landmarks)
├── tp_slam_aruco/
│   ├── aruco_detector_node.py     # Nodo de detección de marcadores ArUco
│   ├── aruco_utils.py             # Utilidades de ArUco sin dependencias ROS
│   ├── graph_slam_node.py         # Nodo principal de Graph SLAM (GTSAM)
│   ├── motion_model.py            # Modelo de odometría (Probabilistic Robotics Ch.5)
│   ├── odometry_node.py           # Nodo de deltas de odometría (auxiliar)
│   └── occupancy_grid_node.py     # 2ª pasada: inverse sensor model + log-odds → /map
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Dependencias externas (pip)

```bash
pip install gtsam numpy pyyaml
pip install opencv-contrib-python   # incluye el módulo aruco
```

Paquetes ROS 2 utilizados: `rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`,
`visualization_msgs`, `tf2_ros`, `cv_bridge`.

---

## Compilación

```bash
cd ~/Robotica/ros_ws
colcon build --packages-select tp_slam_aruco
source install/setup.bash
```

---

## Ejecución — Parte A

El pipeline es de **dos pasadas** sobre el mismo bag:

### 1ª pasada — Graph SLAM (genera la trayectoria corregida)

```bash
# Terminal 1: reproducir el bag con reloj simulado
ros2 bag play laberinto/ --clock

# Terminal 2: lanzar el pipeline SLAM
ros2 launch tp_slam_aruco parte_a_slam.launch.py \
    calibration_file:=/ruta/al/config/camera_tb4_0.yaml \
    trajectory_file:=/ruta/al/runs/trayectoria.json \
    maha_threshold:=5.99 cauchy_k:=1.0
```

Al detener con `Ctrl+C` se guarda automáticamente el JSON con la trayectoria
optimizada, las posiciones de los landmarks y los timestamps de cada keyframe.

**Formato del JSON de salida:**
```json
{
  "trajectory": [
    {"i": 0, "x": 0.0, "y": 0.0, "theta": 0.0, "stamp": 1780877642.5},
    ...
  ],
  "landmarks": {
    "19": {"x": 0.45, "y": 1.23},
    ...
  }
}
```

### 2ª pasada — Mapa de ocupación (proyecta LIDAR sobre la trayectoria)

```bash
# Terminal 1: reproducir el mismo bag
ros2 bag play laberinto/ --clock

# Terminal 2: lanzar la segunda pasada
ros2 launch tp_slam_aruco parte_a_mapa.launch.py \
    trajectory_file:=/ruta/al/runs/trayectoria.json \
    map_output:=/ruta/al/runs/mapa
```

Al detener con `Ctrl+C` exporta `mapa.pgm` + `mapa.yaml` en el prefijo indicado.

### Terminal adicional (opcional): RViz2

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix tp_slam_aruco)/share/tp_slam_aruco/config/rviz_config.rviz
```

---

## Tópicos y TF publicados

| Tópico | Tipo | Descripción |
|--------|------|-------------|
| `/landmarks` | `visualization_msgs/MarkerArray` | Detecciones ArUco en frame cámara |
| `/belief` | `geometry_msgs/PoseStamped` | Pose actual del robot (corregida por el grafo) |
| `/poses_guardadas` | `geometry_msgs/PoseArray` | Todos los keyframes del grafo |
| `/landmarks_opt` | `visualization_msgs/MarkerArray` | Landmarks optimizados en frame mapa |
| `/trajectory_optimized` | `nav_msgs/Path` | Trayectoria completa corregida |
| `/odom_delta` | `nav_msgs/Odometry` | Deltas de odometría (publicado pero no consumido, ver bugs) |
| `/map` | `nav_msgs/OccupancyGrid` | Mapa de grilla — **Parte B, no implementado** |

TF broadcast: `map → odom` a 20 Hz (reloj de pared para independencia del clock del bag).

---

## Lo que está implementado (Parte A)

### `motion_model.py`
Modelo de odometría de tres componentes (δrot1, δtrans, δrot2) según
Probabilistic Robotics, cap. 5. Sin dependencias ROS, testeable de forma aislada.

- `normalize_angle()`: normalización robusta vía `atan2(sin, cos)`.
- `compute_delta(pose_prev, pose_curr)`: extrae los tres deltas de dos poses absolutas.
- `apply_delta(pose_prev, delta)`: propaga una pose hacia adelante.
- `yaw_from_quaternion()`: extrae yaw de un cuaternión.

### `aruco_utils.py`
Funciones de detección y calibración sin ROS.

- `load_camera_calibration(path)`: carga K y coeficientes desde YAML en dos
  formatos (OpenCV y simple). Soporta el modelo `rational_polynomial` de 8
  coeficientes del OAK-D.
- `estimate_marker_poses(corners, ids, ...)`: estimación de pose 3D con
  `solvePnP + SOLVEPNP_IPPE_SQUARE` (óptimo para marcadores planos cuadrados).
  Reemplaza la API `estimatePoseSingleMarkers` deprecada en OpenCV reciente.
- `camera_to_robot_observation(tvec)`: convierte posición en frame cámara a
  (rango, bearing). **No se usa en el pipeline actual** — ver Bug #6.

### `aruco_detector_node.py`
Nodo ROS 2 de detección de marcadores ArUco.

- Soporta OpenCV ≥ 4.5 (API legacy) y ≥ 4.6 (nuevo `ArucoDetector`).
- Obtiene la calibración del tópico `camera_info` (fuente canónica del bag)
  con fallback al YAML de configuración.
- Publica detecciones como `MarkerArray` en `/landmarks` con la pose 3D de
  cada marcador expresada en el frame de la cámara.
- El `frame_id` del marker se hereda del header de la imagen (o se sobreescribe
  con el parámetro `camera_frame`).

### `odometry_node.py`
Consume `tb4_0/odom` y publica deltas de movimiento en `/odom_delta`.
Acumula hasta superar umbrales de traslación (1 cm) o rotación (0.1 rad).
**Su salida no es consumida por ningún nodo** — ver Bug #2.

### `graph_slam_node.py`
Motor principal del SLAM. Construye y optimiza el grafo de poses con GTSAM.

**Keyframes**: se crean cuando el robot recorre ≥ 0.15 m o gira ≥ 0.60 rad
desde el último keyframe. Parámetros ajustables desde el launch.

**Factores del grafo**:
- `PriorFactorPose2`: prior duro en el origen X(0), σ = [0.01 m, 0.01 m, 0.005 rad].
- `BetweenFactorPose2`: restricción de odometría entre keyframes consecutivos
  usando `Pose2.between()` (álgebra de Lie exacta), σ = [0.3 m, 0.3 m, 0.1 rad].
- `BearingRangeFactor2D`: observación de landmark, ruido dependiente de la
  distancia (σ_range = 0.0239 + 0.0315·r², σ_bearing = 0.10 + 0.05·r) con
  kernel robusto de Cauchy.

**Loop closure**: una re-observación del mismo ID se acepta si:
1. El robot se desplazó ≥ 0.20 m desde la última observación del mismo landmark (parallax).
2. El número de factores de ese landmark no superó el límite `MAX_OBS_PER_LANDMARK = 20`.
3. La distancia de Mahalanobis entre la observación y la predicción es < 5.99
   (chi² 2-DOF al 95%). **Ver Bug #5 sobre esta comprobación.**

**Transformación de observaciones**: intenta un lookup TF `camera_frame → base_link`
con fallback a extrínsecos por parámetro (`camera_tx`, `camera_ty`, `camera_yaw`).

**Optimizador**: Levenberg-Marquardt. Por defecto optimiza en cada keyframe
(`optimize_every = 1`).

**TF map→odom**: se difunde desde dos lugares:
- Dentro de `publish_belief()` tras cada optimización.
- Timer independiente a 20 Hz usando `STEADY_TIME` para que funcione aunque
  el reloj simulado del bag esté congelado.

**Exportación**: al recibir `Ctrl+C` guarda `trajectory_file` (JSON) con la
trayectoria optimizada y las posiciones de los landmarks, para ser consumido
por la segunda pasada de mapeo LIDAR (Parte B).

### `launch/parte_a_slam.launch.py`
Lanza los tres nodos activos: `aruco_detector`, `odometry`, `graph_slam`.
Configura `use_sim_time=true` globalmente.

### `config/rviz_config.rviz`
Muestra: grilla de referencia, `LaserScan`, `/belief`, `/poses_guardadas`,
`/landmarks_opt` y odometría cruda.

---

## Diseño de la segunda pasada

### Por qué dos pasadas

En el enunciado (Opción 3) el procedimiento es:
1. **1ª pasada**: construir el grafo de poses corrigiendo la deriva con ArUco (loop closure).
2. **2ª pasada**: con la trayectoria ya fija, proyectar cada barrido LIDAR en la grilla.
   Así las paredes no aparecen duplicadas ni deformadas por la deriva de odometría.

### Sincronización pose ↔ scan (Opción A — offline)

`graph_slam_node` guarda en el JSON el **timestamp** (`stamp`, float en segundos del
bag) de cada keyframe. `occupancy_grid_node` carga ese JSON y, para cada
`LaserScan`, hace búsqueda binaria e **interpolación lineal** de la pose más cercana
en tiempo. Ambas fuentes usan el mismo clock del bag (`use_sim_time=true` + `--clock`),
por lo que los timestamps son directamente comparables entre la 1ª y la 2ª pasada.

### Inverse sensor model + log-odds

```
L_OCC  = +0.85   # log(0.70/0.30): evidencia de impacto
L_FREE = -0.40   # evidencia de celda libre (asimétrico, más conservador al borrar)
L_MIN  = -5.0    # saturación inferior
L_MAX  = +5.0    # saturación superior
```

Para cada rayo válido:
- **Celdas libres** (Bresenham desde el LIDAR hasta impacto-1): `log_odds += L_FREE`
- **Celda de impacto**: `log_odds += L_OCC`

Al exportar: `prob = sigmoid(log_odds)`, luego `prob ≥ 0.6 → 100` (ocupado),
`prob ≤ 0.4 → 0` (libre), resto → `-1` (desconocido).

### Offset del LIDAR (TF real del bag)

```
shell_link → rplidar_link: tx = -0.040 m, ty = 0.0 m
base_link  → shell_link:   tx =  0.000 m, ty = 0.0 m
→ lidar_tx = -0.04 m respecto a base_link
```

Configurado como valor por defecto en `parte_a_mapa.launch.py`.

---

## Bugs y problemas identificados que pueden hacer fallar la Parte A

### Bug 1 — Bag reproducido sin `--clock` (crítico)

El launch configura `use_sim_time=true` para todos los nodos. Si el bag se
reproduce **sin** la flag `--clock`, el tópico `/clock` no se publica y el
reloj simulado de todos los nodos queda congelado en `t=0`. Las suscripciones
con QoS basada en tiempo (especialmente TF) fallan silenciosamente.

El timer de `republish_tf` usa `STEADY_TIME` para evitar esto, pero el resto
del pipeline (suscripciones, timestamps de mensajes) sigue afectado.

**Solución**: siempre reproducir con `ros2 bag play <carpeta> --clock`.

---

### Bug 2 — `odometry_node` publica en un tópico que nadie lee

`odometry_node` calcula deltas y los publica en `/odom_delta`. Sin embargo,
`graph_slam_node` se suscribe directamente a `tb4_0/odom` y recomputa los
deltas internamente con `Pose2.between()`. El tópico `/odom_delta` no tiene
ningún suscriptor. El nodo consume CPU sin contribuir al SLAM.

**Solución**: eliminar `odometry_node` del launch, o hacer que `graph_slam_node`
consuma `/odom_delta` en vez de hacer su propia diferencia de poses.

---

### Bug 3 — QoS incompatible en la suscripción de imagen (crítico)

`aruco_detector_node` suscribe la imagen (`tb4_0/oakd/rgb/preview/image_raw`)
con QoS por defecto (`RELIABLE`, depth=10). Los bags de ROS 2 grabados con
drivers reales típicamente publican con `BEST_EFFORT`. La incompatibilidad de
QoS hace que el nodo **nunca reciba imágenes**, sin mostrar error explícito.

```python
# aruco_detector_node.py:113 — QoS por defecto (RELIABLE):
self.sub = self.create_subscription(
    Image, image_topic, self.image_callback, 10
)
```

La suscripción a `camera_info` sí usa `BEST_EFFORT` correctamente.

**Solución**: verificar el QoS del tópico de imagen en el bag con:
```bash
ros2 bag info <carpeta> | grep image_raw
```
y aplicar el mismo perfil en la suscripción:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
_best_effort = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST, depth=10)
self.sub = self.create_subscription(
    Image, image_topic, self.image_callback, _best_effort)
```

---

### Bug 4 — Extrínsecos cámara→robot incorrectos por defecto (crítico)

Cuando el lookup TF `camera_frame → base_link` falla (lo más probable si no
se publica el árbol TF del robot junto con el bag), el fallback usa:

```python
camera_tx = 0.0   # cámara en el centro exacto del robot
camera_ty = 0.0
camera_yaw = 0.0
```

El OAK-D del TurtleBot4 **no está en el centro** del robot; tiene un offset
hacia adelante. Con los valores por defecto, todos los rangos y bearings de
los landmarks quedan sesgados sistemáticamente, degradando la estimación de
pose y la calidad del loop closure.

**Solución**: medir o buscar los extrínsecos reales del TurtleBot4 #0 y
pasarlos al launch:
```bash
ros2 launch tp_slam_aruco parte_a_slam.launch.py \
    camera_tx:=0.12 camera_ty:=0.0 camera_yaw:=0.0
```
O verificar si el bag incluye TF estático del robot que permita el lookup.

---

### Bug 5 — Innovation gate siempre deshabilitado (medio)

La comprobación de Mahalanobis en `_innovation_gate` consulta
`self.result.atPose2(X(pose_index))`, donde `pose_index` es el índice del
**keyframe recién creado**. En el flujo de `_add_keyframe`, las observaciones
se procesan **antes** de llamar a `optimize()`, por lo tanto `self.result`
todavía no contiene `X(pose_index)`. El `try/except` captura la excepción y
retorna `True` (el gate pasa), anulando la comprobación.

```python
# graph_slam_node.py:204-206 — observaciones ANTES de optimize():
for lm_id, (rng, brg) in self.pending_detections.items():
    self._add_observation(i, lm_id, rng, brg)   # gate siempre True para X(i)
self.pending_detections.clear()
# ...
if (self.pose_count % self.optimize_every) == 0:
    self.optimize()   # aquí se genera self.result con X(i)
```

Resultado: cualquier re-observación —incluyendo las outliers— se agrega al
grafo sin filtrar.

**Solución**: ejecutar una optimización parcial antes de las observaciones, o
evaluar el gate usando la estimación inicial (`self.initial`) en lugar de
`self.result`.

---

### Bug 6 — `camera_to_robot_observation()` es código muerto

La función `camera_to_robot_observation` en `aruco_utils.py` no se invoca en
ningún lugar del pipeline. La conversión real de coordenadas cámara→base_link
se hace en `landmarks_cb` vía `_marker_to_base_xy()`, con convención de ejes
diferente. La función en `aruco_utils.py` puede inducir a error al querer
reutilizarla.

---

### Bug 7 — `MAX_OBS_PER_LANDMARK = 20` puede agotar el loop closure (medio)

En un laberinto con múltiples vueltas, un marker visto frecuentemente alcanza
el límite de 20 factores y deja de contribuir al cierre de lazos. Si los
marcadores más importantes (esquinas del laberinto) se saturan antes de que el
robot complete todas las vueltas, la corrección de deriva deja de funcionar.

```python
# graph_slam_node.py:297
MAX_OBS_PER_LANDMARK = 20
```

**Solución**: aumentar el límite (p. ej. 50) o implementar marginalización
selectiva para reutilizar el presupuesto en re-observaciones posteriores.

---

### Bug 8 — `camera_frame` no propagado al `graph_slam_node` vía parámetro

El launch pasa `camera_frame` al nodo `aruco_detector` (para el `frame_id`
del marker), pero **no** lo pasa a `graph_slam_node`. El nodo de SLAM usa el
`frame_id` del mensaje directamente para el TF lookup, lo cual funciona siempre
que el aruco detector lo configure correctamente. Sin embargo, `graph_slam_node`
declara el parámetro `camera_frame` internamente pero jamás se le asigna desde
el launch, quedando con el valor por defecto vacío.

---

### Bug 9 — `optimize()` sin protección ante fallos de GTSAM (bajo)

El método `optimize()` llama a `optimizer.optimize()` sin `try/except`. Un
grafo degenero (p. ej. un landmark observado una sola vez con una pose muy
incierta) puede hacer que GTSAM lance una excepción y termine el nodo.

**Solución**: envolver en `try/except` y loguear el error sin interrumpir el
proceso:
```python
try:
    self.result = optimizer.optimize()
except Exception as e:
    self.get_logger().error(f'optimize() falló: {e}')
    return
```

---

## Resumen de estado

| Componente | Estado |
|---|---|
| Modelo de odometría (`motion_model.py`) | ✅ Completo |
| Utilidades ArUco sin ROS (`aruco_utils.py`) | ✅ Completo |
| Detector ArUco ROS 2 (`aruco_detector_node.py`) | ✅ Completo |
| Nodo de deltas de odometría (`odometry_node.py`) | ✅ Completo (salida no consumida) |
| Graph SLAM con GTSAM (`graph_slam_node.py`) | ✅ Completo (con bugs documentados) |
| Launch 1ª pasada (`parte_a_slam.launch.py`) | ✅ Funcional |
| Launch 2ª pasada (`parte_a_mapa.launch.py`) | ✅ Funcional |
| Calibración cámara OAK-D TB4 #0 | ✅ Presente |
| Configuración RViz2 | ✅ Presente |
| Grilla de ocupación (`occupancy_grid_node.py`) | ✅ Implementado |
| Exportación `.pgm` + `.yaml` | ✅ Implementado |
