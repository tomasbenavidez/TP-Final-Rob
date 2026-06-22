# TP Final de Robótica Autónoma

Implementación ROS 2 del flujo completo del trabajo final:

1. **Parte A — SLAM y mapeo:** procesa un rosbag real del TurtleBot4, estima la trayectoria con Graph SLAM y ArUco, y genera una grilla de ocupación.
2. **Parte B — navegación autónoma:** carga un mapa estático, localiza el TurtleBot3 simulado con MCL y landmarks virtuales, planifica con A* y ejecuta el recorrido mediante una máquina de estados.

Las partes se ejecutan por separado. El artefacto que las conecta es el mapa `map.yaml` + `map.pgm`.

## Estructura

```text
TP-Final-Rob/
├── mapas/                         # mapa de referencia usado por Parte B
├── docs/parte_b/                  # implementación, ejecución y resultados
├── tp_final_ws/
│   ├── bags/                      # rosbags de Parte A (no versionados)
│   └── src/
│       ├── tp_slam_aruco/         # Parte A
│       ├── tp_slam_interfaces/    # mensajes ROS compartidos
│       └── tp_b_navigation/       # Parte B
└── AGENTS.md                      # contexto técnico para agentes de desarrollo
```

## Requisitos

- ROS 2 Humble y `colcon`.
- Parte A: `numpy<2`, GTSAM, PyYAML, OpenCV contrib, SciPy y `cv_bridge`.
- Parte B: Gazebo, TurtleBot3 y el paquete de simulación `turtlebot3_custom_simulation` provisto por la cátedra.

Dependencias Python principales:

```bash
python3 -m pip install "numpy<2" gtsam pyyaml opencv-contrib-python scipy
```

## Build

Todos los comandos siguientes parten de la raíz del repositorio clonado:

```bash
cd tp_final_ws
colcon build --packages-select tp_slam_interfaces tp_slam_aruco tp_b_navigation
source install/setup.bash        # bash
# source install/setup.zsh       # zsh
```

Durante el build, el mapa de `mapas/` se copia al directorio instalado de `tp_b_navigation`. Los launch lo encuentran mediante el índice de paquetes de ROS; no dependen de la ubicación del clon.

## Parte A — generar el mapa

La opción implementada usa dos reproducciones del rosbag.

### Primera pasada: ArUco + Graph SLAM

Terminal 1, desde `tp_final_ws/`:

```bash
source install/setup.bash
ros2 bag play bags/laberinto --clock
```

Terminal 2:

```bash
source install/setup.bash
ros2 launch tp_slam_aruco parte_a_slam.launch.py \
  calibration_file:="$(pwd)/src/tp_slam_aruco/config/camera_tb4_0.yaml" \
  trajectory_file:=/tmp/trayectoria.json \
  use_bag_tf:=true
```

Al finalizar con `Ctrl+C`, el nodo guarda `/tmp/trayectoria.json`.

Salidas principales: `/aruco_detections`, `/belief`, `/poses_guardadas`, `/trajectory_optimized`, `/landmarks` y TF `map → odom`.

### Segunda pasada: grilla de ocupación

Terminal 1:

```bash
source install/setup.bash
ros2 bag play bags/laberinto --clock
```

Terminal 2:

```bash
source install/setup.bash
ros2 launch tp_slam_aruco parte_a_mapa.launch.py \
  trajectory_file:=/tmp/trayectoria.json \
  map_output:=/tmp/mapa
```

La salida es `/tmp/mapa.pgm` + `/tmp/mapa.yaml`.

Para usar un mapa regenerado como default de Parte B, reemplazá `mapas/map.pgm` y `mapas/map.yaml` y repetí `colcon build --packages-select tp_b_navigation`. Para probarlo sin reemplazar archivos, pasalo directamente con `map_yaml:=/tmp/mapa.yaml`.

## Parte B — navegar dentro del mapa

Parte B se ejecuta en Gazebo. Todos sus nodos usan tiempo simulado.

Terminal 1:

```bash
source tp_final_ws/install/setup.bash
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
# Para probar obstáculos no mapeados:
# ros2 launch turtlebot3_custom_simulation custom_casa_obs.launch.py
```

Terminal 2:

```bash
source tp_final_ws/install/setup.bash
ros2 launch tp_b_navigation parte_b.launch.py
```

En RViz:

1. Usá **2D Pose Estimate** para publicar `/initialpose`.
2. Usá **2D Goal Pose** para publicar `/goal_pose`.
3. El sistema localiza con MCL, publica `/plan`, sigue el recorrido, evita obstáculos nuevos y alcanza la orientación final.

Nodos de Parte B:

- `map_loader`
- `landmark_publisher`
- `landmark_sensor`
- `mcl_localization`
- `global_planner`
- `obstacle_monitor`
- `state_machine`

La guía detallada, incluido el setup alternativo para RoboStack/macOS, está en [`docs/parte_b/02_guia_ejecucion.md`](docs/parte_b/02_guia_ejecucion.md).

## Tests

Con el entorno ROS cargado:

```bash
python3 -m pytest tp_final_ws/src/tp_slam_aruco/test -q
python3 -m pytest tp_final_ws/src/tp_b_navigation/test -q
```

El smoke test de Parte A con rosbag se habilita explícitamente:

```bash
RUN_ROS_SMOKE=1 python3 -m pytest tp_final_ws/src/tp_slam_aruco/test/test_ros_smoke.py -q
```

## Contratos importantes

- Parte A y Parte B no deben lanzarse simultáneamente con la configuración actual.
- Ambas etapas pueden publicar `map → odom`, pero en ejecuciones diferentes.
- `/landmarks` tiene contratos distintos: ArUco optimizados en Parte A y landmarks virtuales conocidos en Parte B.
- `state_machine` es el único nodo de Parte B autorizado a publicar `/cmd_vel`.
- Para usar un mapa alternativo: `ros2 launch tp_b_navigation parte_b.launch.py map_yaml:=/ruta/map.yaml`.
