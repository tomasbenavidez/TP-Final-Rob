# AGENTS.md — TP Final de Robótica Autónoma

## Objetivo del repositorio

Este repositorio implementa un sistema ROS 2 en etapas:

1. **Parte A:** Graph SLAM con ArUco sobre un rosbag real del TurtleBot4 y generación posterior de una grilla de ocupación.
2. **Parte B:** localización probabilística y navegación autónoma de un TurtleBot3 simulado dentro de un mapa estático.
3. **Parte C:** exploración informativa, detección de conos rojos y despliegue sim-to-real.

La integración normal es secuencial: Parte A produce `map.yaml` + `map.pgm`; Parte B carga ese mapa. No asumir que A y B se ejecutan simultáneamente.

## Estructura relevante

```text
mapas/                              mapa estático de referencia
docs/parte_b/                       documentación y resultados de Parte B
docs/parte_c/                       documentación y ejecución de Parte C
tp_final_ws/src/tp_slam_aruco/      paquete de Parte A
tp_final_ws/src/tp_slam_interfaces/ mensajes ROS propios
tp_final_ws/src/tp_b_navigation/    paquete de Parte B
tp_final_ws/src/tp_c_mission/       percepción y supervisor de Parte C
```

## Parte A

Launch principales:

- `parte_a_slam.launch.py`: reproduce detecciones ArUco + odometría, optimiza el grafo y guarda la trayectoria.
- `parte_a_mapa.launch.py`: usa la trayectoria corregida y odometría densa para proyectar el LIDAR y exportar el mapa.

Nodos principales:

- `tf_bridge_node`: república TF del rosbag.
- `aruco_detector_node`: detecta y filtra ArUco.
- `graph_slam_node`: estima poses y landmarks, publica `map → odom`.
- `occupancy_grid_node`: genera `map.pgm` + `map.yaml` en la segunda pasada.

Invariantes importantes:

- Opción 3 de la consigna: Graph SLAM es obligatorio.
- La segunda pasada compone corrección SLAM con odometría densa; no reemplazarla por interpolación simple entre keyframes.
- El LIDAR real del bag tiene `lidar_yaw=π/2` y `lidar_tx=-0.04`.
- La prioridad para cámara → base es TF real del bag y luego fallback numérico.
- Los diagnósticos se escriben por defecto en `/tmp`, no en rutas del checkout.

## Parte B

`parte_b.launch.py` inicia siete nodos:

| Nodo | Responsabilidad |
|---|---|
| `map_loader` | Carga y publica `/map` con QoS latched |
| `landmark_publisher` | Publica 36 landmarks virtuales conocidos |
| `landmark_sensor` | Simula range/bearing, FOV, oclusión y ruido |
| `mcl_localization` | Filtro de partículas y TF `map → odom` |
| `global_planner` | A* 8-conexo, inflado y simplificación de ruta |
| `obstacle_monitor` | Detecta obstáculos nuevos comparando `/scan` con `/map` |
| `state_machine` | Planificación, seguimiento, evasión, alineación y `/cmd_vel` |

Invariantes importantes:

- `state_machine` es el único productor de `/cmd_vel` dentro de Parte B.
- `landmark_sensor` usa `odom` como verdad simulada para evitar realimentar MCL con su estimación.
- `mcl_localization` es el productor de `map → odom` en Parte B.
- No cambiar parámetros de MCL, A*, pure pursuit o evasión sin una prueba que justifique el cambio.
- Los mapas instalados se resuelven desde `share/tp_b_navigation/maps`; no introducir rutas personales.

## Parte C

Launch principales:

- `parte_c_sim.launch.py`: misión completa en `custom_casa` o `custom_casa_obs`.
- `parte_c_bag.launch.py`: calibración/regresión del detector con rosbag de visión.
- `parte_c_real.launch.py`: mapa y landmarks de Parte A, ArUco real y TurtleBot4.

Invariantes importantes:

- `tp_c_mission` no publica `/cmd_vel`; envía `/mission_goal` a `state_machine`.
- MCL continúa siendo el único productor de `map → odom` durante Parte C.
- El perfil real no inicia `landmark_sensor`; usa `/observed_landmark_ids` desde `aruco_mcl_adapter`.
- La detección del cono se valida temporalmente y produce una pose en `map`, no un comando directo.
- La aproximación al cono se valida con la grilla y A*; no asumir camino libre por visibilidad de cámara.
- El costmap dinámico y `custom_casa_obs2` quedan fuera de alcance.

## Contratos que difieren entre etapas

- `/landmarks` de Parte A es un `MarkerArray` de ArUco optimizados.
- `/landmarks` de Parte B es un `PoseArray` de referencias virtuales conocidas.
- `graph_slam_node` y `mcl_localization` publican alternativamente `map → odom`.
- Parte A usa tópicos `tb4_0/*` y frame `base_link`; Parte B usa `/scan`, `/odom` y `base_footprint` de Gazebo.

Estos contratos son válidos porque las etapas se lanzan por separado. No unificarlos como refactor mecánico.

## Comandos portables

Desde la raíz del repositorio:

```bash
cd tp_final_ws
colcon build --packages-select tp_slam_interfaces tp_slam_aruco \
  tp_b_navigation tp_c_mission turtlebot3_custom_simulation
source install/setup.bash
```

Parte A:

```bash
ros2 bag play bags/laberinto --clock
ros2 launch tp_slam_aruco parte_a_slam.launch.py \
  calibration_file:="$(pwd)/src/tp_slam_aruco/config/camera_tb4_0.yaml" \
  trajectory_file:=/tmp/trayectoria.json use_bag_tf:=true
```

Parte B, desde la raíz del repositorio y con Gazebo en otra terminal:

```bash
source tp_final_ws/install/setup.bash
ros2 launch tp_b_navigation parte_b.launch.py
```

Parte C:

```bash
source tp_final_ws/install/setup.bash
ros2 launch tp_c_mission parte_c_sim.launch.py world:=casa
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
```

Para RoboStack/macOS se puede usar `source docs/parte_b/scripts/setup_parte_b.sh`; el script deriva la raíz del repositorio automáticamente.

## Verificación antes de entregar cambios

```bash
python3 tp_final_ws/src/tp_b_navigation/test/test_portable_paths.py -v
python3 -m compileall -q tp_final_ws/src/tp_b_navigation tp_final_ws/src/tp_slam_aruco tp_final_ws/src/tp_c_mission
git diff --check
```

Con ROS 2 disponible, ejecutar además:

```bash
cd tp_final_ws
colcon build --packages-select tp_slam_interfaces tp_slam_aruco \
  tp_b_navigation tp_c_mission turtlebot3_custom_simulation
source install/setup.bash
python3 -m pytest src/tp_slam_aruco/test src/tp_b_navigation/test src/tp_c_mission/test -q
```

## Límites de modificación

- Preservar cambios ajenos y archivos no versionados.
- No editar algoritmos estables como parte de tareas documentales o de portabilidad.
- No agregar rutas absolutas dependientes del usuario.
- Mantener parámetros configurables mediante launch o YAML.
- Actualizar README y documentación cuando cambien contratos de ejecución.
