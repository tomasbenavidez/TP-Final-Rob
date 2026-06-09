# TP Final Rob - Parte A

Este repositorio contiene el trabajo en curso para la **Parte A** del TP final
de Robotica Autonoma: usar un rosbag del TurtleBot4, detectar landmarks ArUco,
construir una trayectoria corregida con Graph SLAM y proyectar el LIDAR sobre
esa trayectoria para generar un mapa de ocupacion.

La rama actual queda como punto de continuidad. No debe considerarse una entrega
cerrada: todavia hay bugs conocidos, especialmente en la carga/confirmacion de
landmarks.

## Estado de la rama

- **Primera pasada:** deteccion ArUco + Graph SLAM para generar
  `trayectoria.json`.
- **Segunda pasada:** trayectoria optimizada + LIDAR para generar `mapa.pgm` y
  `mapa.yaml`.
- **Visualizacion:** `parte_a_slam.launch.py` puede levantar RViz como parte del
  pipeline.
- **Pendiente principal:** revisar por que los landmarks no terminan entrando al
  grafo de forma confiable.

Para mas contexto de continuidad, ver [docs/parte_a_handoff.md](docs/parte_a_handoff.md).

## Estructura relevante

```text
tp_final_ws/
├── bags/
├── src/
│   ├── tp_slam_interfaces/
│   │   └── msg/
│   └── tp_slam_aruco/
│       ├── config/
│       ├── launch/
│       ├── test/
│       └── tp_slam_aruco/
```

`tp_slam_interfaces` define los mensajes internos usados para pasar
observaciones visuales del detector ArUco al nodo de SLAM. `tp_slam_aruco`
contiene los nodos, launch files, configuracion y tests.

## Requisitos

Se asume una instalacion funcional de ROS 2 con `cv_bridge` disponible.

Dependencias Python usadas por el paquete:

```bash
pip install "numpy<2" gtsam pyyaml opencv-contrib-python
python -c "import cv_bridge, cv2, gtsam, numpy; print(numpy.__version__)"
```

## Build

Desde la raiz del repositorio:

```bash
cd tp_final_ws
colcon build --packages-select tp_slam_interfaces tp_slam_aruco
source install/setup.bash
```

## Parte A - Primera pasada

Terminal 1:

```bash
cd tp_final_ws
source install/setup.bash
ros2 bag play <ruta_al_bag> --clock
```

Terminal 2:

```bash
cd tp_final_ws
source install/setup.bash
ros2 launch tp_slam_aruco parte_a_slam.launch.py \
  calibration_file:=<ruta_al_yaml_de_calibracion> \
  trajectory_file:=<ruta_salida>/trayectoria.json \
  use_bag_tf:=true
```

Salida esperada al cortar el launch con `Ctrl+C`:

```text
<ruta_salida>/trayectoria.json
```

Topicos importantes para mirar durante esta pasada:

- `/landmark_observations`
- `/landmark_detection_stats`
- `/belief`
- `/poses_guardadas`
- `/landmarks_opt`
- `/trajectory_optimized`

## Parte A - Segunda pasada

Terminal 1:

```bash
cd tp_final_ws
source install/setup.bash
ros2 bag play <ruta_al_bag> --clock
```

Terminal 2:

```bash
cd tp_final_ws
source install/setup.bash
ros2 launch tp_slam_aruco parte_a_mapa.launch.py \
  trajectory_file:=<ruta_salida>/trayectoria.json \
  map_output:=<ruta_salida>/mapa
```

Salida esperada al cortar con `Ctrl+C`:

```text
<ruta_salida>/mapa.pgm
<ruta_salida>/mapa.yaml
```

## Handoff

Quien continue esta rama deberia empezar por validar el flujo de landmarks antes
de ajustar el mapeo LIDAR. En particular:

- Confirmar que el detector publica observaciones en `/landmark_observations`.
- Confirmar que `graph_slam_node` recibe esas observaciones y las convierte en
  candidatos.
- Revisar los filtros de edad, parallax y reproyeccion antes de relajar o
  endurecer parametros.
- Verificar que `camera_info` y el TF camara -> `base_link` esten disponibles
  durante la reproduccion del bag.
