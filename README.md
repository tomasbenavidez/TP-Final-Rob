# TP Final Robótica — Opción 3 (Graph SLAM con ArUco + LIDAR)

SLAM basado en marcadores ArUco (cámara) y LIDAR sobre datos reales del
**TurtleBot4** (RosBag), con **Graph SLAM** y cierre de lazo. Backend de
optimización: **GTSAM**. Nodos en **Python (rclpy)**.

> Estado: **Parte A en desarrollo.** Esqueleto completo + odometría y
> detección ArUco funcionales. Graph SLAM (F4) y occupancy grid (F5) en
> progreso.

---

## Estructura del repo

```
src/tp_slam_aruco/
├── package.xml                 # manifiesto ROS2
├── setup.py                    # registro de nodos ejecutables
├── launch/
│   └── parte_a_slam.launch.py  # orquesta los nodos de Parte A
├── config/                     # calibración de cámara, .rviz, parámetros
└── tp_slam_aruco/
    ├── motion_model.py         # [lib] matemática de deltas de odometría (sin ROS)
    ├── aruco_utils.py          # [lib] calibración + estimación de pose (sin ROS)
    ├── odometry_node.py        # nodo: odom cruda -> deltas
    ├── aruco_detector_node.py  # nodo: cámara -> landmarks ArUco
    ├── graph_slam_node.py      # nodo: grafo GTSAM + loop closure (F4, esqueleto)
    └── occupancy_grid_node.py  # nodo: 2da pasada -> mapa .pgm/.yaml (F5, esqueleto)
```

**Idea de diseño:** la matemática pura vive en módulos `*_utils` / `*_model`
sin dependencias de ROS, así se puede testear aislada. Los `*_node.py` solo
envuelven esa lógica en la maquinaria de ROS (suscripciones, publicaciones).

---

## Flujo de datos (Parte A)

```
RosBag (tb4_0/odom, cámara, tb4_0/scan)
   ├─► aruco_detector_node ──► observaciones de landmarks (id, rango, bearing)
   ├─► odometry_node ────────► deltas (δrot1, δtrans, δrot2)
   └──────────► graph_slam_node ──► trayectoria corregida + landmarks
                      └──► occupancy_grid_node ──► mapa .pgm/.yaml
```

---

## Setup del entorno

> Por ahora **sin Docker.** Cada integrante levanta su propio entorno.
> Recordá: los `ros_ws` pueden estar en rutas distintas en cada máquina;
> lo único compartido por Git es `src/`. `build/`, `install/` y `log/`
> están en `.gitignore` y se generan localmente con `colcon build`.

### 1. Requisitos base
- ROS 2 Humble + dependencias de TurtleBot4
- Ubuntu 22.04 nativo, o RoboStack (Conda) en Mac/Linux

### 2. Dependencias Python (NO son paquetes ROS, van por pip)
```bash
pip install gtsam numpy pyyaml
# OpenCV con el módulo aruco (contrib):
pip install opencv-contrib-python
```

### 3. Clonar y compilar
```bash
cd ~/<tu_ros_ws>/src        # la ruta de tu workspace es local, no importa cuál sea
git clone <url_del_repo> .
cd ~/<tu_ros_ws>
colcon build --packages-select tp_slam_aruco
source install/setup.bash
```

---

## Uso

```bash
# Terminal 1 — reproducir el bag del laberinto (provisto por la cátedra)
ros2 bag play <carpeta_del_bag>

# Terminal 2 — lanzar el pipeline de SLAM
ros2 launch tp_slam_aruco parte_a_slam.launch.py \
    calibration_file:=src/tp_slam_aruco/config/tb4_0_camera.yaml
```

### Probar nodos sueltos
```bash
ros2 run tp_slam_aruco odometry
ros2 run tp_slam_aruco aruco_detector --ros-args -p calibration_file:=<ruta>
```

### Testear la matemática sin ROS
```bash
cd src/tp_slam_aruco/tp_slam_aruco
python3 -c "from motion_model import compute_delta, apply_delta; ..."
```

---

## Pendientes inmediatos (ver tablero en Notion)
- **F1** — colocar la calibración real del TB4 #0 en `config/` y validar
  contra `aruco_estimation.zip`.
- **F2** — confirmar el diccionario ArUco correcto y el nombre del tópico
  de cámara del bag (ajustar `image_topic`).
- **F4** — implementar la construcción y optimización del grafo en GTSAM.
- **F5** — implementar la segunda pasada y la exportación del mapa.
