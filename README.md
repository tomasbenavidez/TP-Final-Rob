# TP Final Robótica — Opción 3 (Graph SLAM con ArUco + LIDAR)

SLAM basado en marcadores ArUco (cámara) y LIDAR sobre datos reales del
**TurtleBot4** (RosBag), con **Graph SLAM** y cierre de lazo. Backend de
optimización: **GTSAM**. Nodos en **Python (rclpy)**.

> Estado: **Parte A en desarrollo.** Percepción ArUco calibrada y validada
> contra ground truth de la cátedra. Graph SLAM (F4) implementado y validado
> en seco; falta correrlo sobre el bag del laberinto. Occupancy grid (F5) en
> progreso.

---

## Estructura del repo

```
src/tp_slam_aruco/
├── package.xml                 # manifiesto ROS2
├── setup.py                    # registro de nodos ejecutables
├── launch/
│   └── parte_a_slam.launch.py  # orquesta los nodos de Parte A
├── config/
│   └── camera_tb4_0.yaml       # calibración real de la cámara del TB4 #0
└── tp_slam_aruco/
    ├── motion_model.py         # [lib] matemática de deltas de odometría (sin ROS)
    ├── aruco_utils.py          # [lib] calibración + estimación de pose (sin ROS)
    ├── odometry_node.py        # nodo: odom cruda -> deltas
    ├── aruco_detector_node.py  # nodo: cámara -> landmarks ArUco
    ├── graph_slam_node.py      # nodo: grafo GTSAM + loop closure (F4, implementado)
    └── occupancy_grid_node.py  # nodo: 2da pasada -> mapa .pgm/.yaml (F5, esqueleto)
```

**Idea de diseño:** la matemática pura vive en módulos `*_utils` / `*_model`
sin dependencias de ROS, así se puede testear aislada. Los `*_node.py` solo
envuelven esa lógica en la maquinaria de ROS (suscripciones, publicaciones).

---

## Flujo de datos (Parte A)

```
RosBag (tb4_0/odom, cámara, tb4_0/scan)
   ├─► aruco_detector_node ──► observaciones de landmarks (MarkerArray en /landmarks)
   ├─► odometry_node ────────► deltas (δrot1, δtrans, δrot2)
   └──────────► graph_slam_node ──► trayectoria corregida + landmarks
                      └──► occupancy_grid_node ──► mapa .pgm/.yaml
```

---

## Parámetros reales del bag (CONFIRMADOS)

Verificados contra el `metadata.yaml` y el bag `aruco_estimation`:

| Qué | Valor |
|-----|-------|
| Tópico cámara (RGB) | `/tb4_0/oakd/rgb/preview/image_raw` |
| Tópico `camera_info` | `/tb4_0/oakd/rgb/preview/camera_info` |
| Tópico odometría | `/tb4_0/odom` |
| Tópico LIDAR | `/tb4_0/scan` |
| Diccionario ArUco | `DICT_4X4_50` |
| Tamaño del marcador | `0.0889 m` |
| Plataforma | TurtleBot4 número **0** |

### ⚠️ QoS — leer antes de armar cualquier nodo nuevo

Los tópicos del bag tienen perfiles QoS **distintos** y esto causa fallos
silenciosos ("New publisher discovered ... incompatible QoS. No messages will
be received"):

| Tópico | Reliability |
|--------|-------------|
| `tb4_0/odom`, `tb4_0/imu` | **BEST_EFFORT** |
| cámara, `tb4_0/scan`, `tf`, `tf_static` | RELIABLE |

Cualquier suscriptor a **odom** o **imu** debe declarar QoS BEST_EFFORT:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
odom_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=50)
self.create_subscription(Odometry, odom_topic, self.odom_cb, odom_qos)
```

Si un nodo "no recibe nada", revisar QoS **primero** — es la causa más común.

---

## Estado de la calibración y el modelo de medición (F1/F2 — CERRADO)

La calibración del TB4 #0 (matriz K + primeros 5 coeficientes de distorsión)
está en `config/camera_tb4_0.yaml`. **Validada** corriendo el detector sobre
`aruco_estimation` y comparando contra el ground truth de la cátedra:

| Z medido | real (cátedra) | esperado (cátedra) | error |
|----------|----------------|--------------------|-------|
| 0.268 | 0.30 | 0.28 | 3.2 cm |
| 0.734 | 0.695 | 0.72 | 3.9 cm |
| 1.062 | 1.01 | 1.04 | 5.2 cm |
| 1.598 | 1.49 | 1.52 | 10.8 cm |

El error crece con la distancia (esperable: menos píxeles → solvePnP peor
condicionado). De ahí sale el **modelo de ruido de medición por rango**:

```
σ_z(Z) = 0.0239 + 0.0315 · Z²    [metros]
```

Este σ se aplica como covarianza del factor bearing-range en GTSAM, así las
detecciones lejanas pesan menos en la optimización del grafo.

> Nota: `aruco_estimation` es un bag de **calibración**: el robot está quieto
> y lo que se mueve es el marcador. Sirve para caracterizar la medición, pero
> **no** para ejercitar el grafo de poses (no hay trayectoria). Para eso se
> usa `laberinto.zip`.

---

## Setup del entorno

> Por ahora **sin Docker.** Cada integrante levanta su propio entorno.
> Recordá: los `ros_ws` pueden estar en rutas distintas en cada máquina;
> lo único compartido por Git es `src/`. `build/`, `install/`, `log/` y los
> bags (`bags/`) están en `.gitignore` y se generan/descargan localmente.

### 1. Requisitos base
- ROS 2 Humble + dependencias de TurtleBot4
- Ubuntu 22.04 nativo, o RoboStack (Conda) en Mac/Linux

### 2. Dependencias Python (NO son paquetes ROS, van por pip)
```bash
pip install gtsam numpy pyyaml opencv-contrib-python --break-system-packages
```

> En Ubuntu 22.04 con Python de sistema, pip pide `--break-system-packages`.
> **Los tres integrantes** necesitan estas deps: `colcon build` NO instala
> paquetes de pip, así que sin esto el nodo de SLAM tira `ModuleNotFoundError:
> No module named 'gtsam'`.

#### Compatibilidad de versiones (ojo)
- **OpenCV ArUco:** el código soporta la API vieja (< 4.7, funciones sueltas
  como `cv2.aruco.detectMarkers`) y la nueva (≥ 4.7, clase `ArucoDetector`)
  vía un fallback con `hasattr(cv2.aruco, 'ArucoDetector')`. Funciona en
  ambas, pero confirmá tu versión con
  `python3 -c "import cv2; print(cv2.__version__)"`.
- **numpy:** ROS Humble y GTSAM andan con numpy 1.26.x. Versiones muy nuevas
  de opencv-contrib piden numpy ≥ 2; si aparecen conflictos, NO actualices
  numpy a ciegas (rompe ROS) — fijá una versión compatible.

### 3. Clonar y compilar
```bash
cd ~/<tu_ros_ws>/src        # la ruta de tu workspace es local, no importa cuál sea
git clone <url_del_repo> .
cd ~/<tu_ros_ws>
colcon build --packages-select tp_slam_aruco
source install/setup.bash
```

### 4. Bags
Los bags **no se versionan** (pesados, son datos). Estructura local sugerida:
```
~/<tu_ros_ws>/
├── src/                    # esto va a Git
└── bags/                   # esto NO va a Git (en .gitignore)
    ├── aruco_estimation/   # metadata.yaml + .db3 JUNTOS en la carpeta
    └── laberinto/
```
`ros2 bag play` recibe **la carpeta**, no el `.db3` suelto. Verificá con
`ros2 bag info bags/<carpeta>`.

---

## Uso

```bash
# Terminal 1 — reproducir el bag (provisto por la cátedra)
ros2 bag play bags/<carpeta_del_bag>

# Terminal 2 — lanzar el pipeline de SLAM
ros2 launch tp_slam_aruco parte_a_slam.launch.py \
    calibration_file:=src/tp_slam_aruco/config/camera_tb4_0.yaml
```

### Probar nodos sueltos
```bash
ros2 run tp_slam_aruco odometry
ros2 run tp_slam_aruco aruco_detector --ros-args \
    -p calibration_file:=src/tp_slam_aruco/config/camera_tb4_0.yaml
ros2 run tp_slam_aruco graph_slam_node --ros-args \
    -p kf_dist:=0.15 -p kf_angle:=0.20
```

> Sintaxis de parámetros: `--ros-args -p nombre:=valor` (con `--ros-args` y
> con `:=`). Sin eso, el nodo usa los valores por defecto.

#### Parámetros de `graph_slam_node`
- `odom_topic` (def. `tb4_0/odom`)
- `landmarks_topic` (def. `/landmarks`)
- `kf_dist` (def. `0.15` m) — distancia mínima para crear un keyframe nuevo
- `kf_angle` (def. `0.20` rad) — giro mínimo para crear un keyframe nuevo
- `optimize_every` (def. `1`) — optimizar cada N keyframes

> El nodo usa **keyframes**: no crea un nodo de pose por cada mensaje de odom,
> sino solo cuando el robot se movió lo suficiente. Mantiene el grafo chico y
> escalable para el laberinto.

### Testear la matemática sin ROS
```bash
cd src/tp_slam_aruco/tp_slam_aruco
python3 -c "from motion_model import compute_delta, apply_delta; ..."
```

---

## Estado por tarea

- **F1** ✅ — calibración real del TB4 #0 en `config/camera_tb4_0.yaml`,
  validada contra `aruco_estimation` (tabla de arriba).
- **F2** ✅ — diccionario (`DICT_4X4_50`) y tópico de cámara confirmados.
- **F3** ⏳ — ruido de odometría (alpha1..4): falta calibrar. Por ahora el
  grafo usa un σ de odom placeholder (`[0.05, 0.05, 0.03]`).
- **F4** 🔨 — `graph_slam_node` implementado: prior, factores de odometría
  (`BetweenFactorPose2`), factores de medición (`BearingRangeFactor2D`) con
  ruido por distancia, loop closure automático al re-observar un id, y
  publicación de `/belief`, `/poses_guardadas`, `/landmarks_opt`. Validado en
  seco (el grafo converge). **Falta correrlo sobre el bag del laberinto** —
  ahí aparecen la trayectoria real y los loop closures.
- **F5** ⏳ — `occupancy_grid_node`: segunda pasada proyectando el LIDAR sobre
  la trayectoria corregida → mapa `.pgm`/`.yaml`.

### Pendientes inmediatos
1. Correr F4 sobre `laberinto.zip` y verificar trayectoria + loop closures.
2. Evaluar si la asociación detección→keyframe (hoy "última detección vista")
   aguanta el laberinto, o si hace falta `ApproximateTimeSynchronizer` sobre
   timestamps.
3. Caracterizar σ del bearing (hoy fijo en 0.05 rad) y el ruido de odom (F3).
4. Implementar F5 y exportar el mapa final.
