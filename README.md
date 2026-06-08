# TP Final Rob - Parte A

Este repo hoy esta enfocado en la **Parte A** del TP final de Robotica Autonoma:
tomar un rosbag real del TurtleBot4, hacer **Graph SLAM con ArUco**, y despues
proyectar el LIDAR sobre la trayectoria corregida para generar un mapa.

La ruta real del workspace en este repo es:

```bash
/Users/franco/TP-Final-Rob/tp_final_ws
```

## Que corre hoy

- **1ra pasada:** deteccion ArUco + Graph SLAM -> `trajectory.json`
- **2da pasada:** trayectoria fija + LIDAR -> `mapa.pgm` y `mapa.yaml`
- **RViz:** se abre aparte, no lo lanza `parte_a_slam.launch.py`

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

- `/belief`
- `/poses_guardadas`
- `/landmarks_opt`
- `/trajectory_optimized`
- TF `map -> odom`

Cuando frenes el launch con `Ctrl+C`, se guarda:

```bash
/tmp/trayectoria.json
```

## RViz

RViz es un paso manual.

```bash
cd /Users/franco/TP-Final-Rob/tp_final_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d /Users/franco/TP-Final-Rob/tp_final_ws/src/tp_slam_aruco/config/rviz_config.rviz
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
