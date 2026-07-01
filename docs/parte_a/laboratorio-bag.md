# Bag `laboratorio`: mapa y localizacion offline

Objetivo: usar `tp_final_ws/bags/laboratorio` para generar un mapa de Parte A y
despues reproducir el mismo bag para probar MCL contra ese mapa.

El bag esta grabado con namespace `/tb4_1`. Por eso todos los comandos deben
usar `robot_namespace:=tb4_1`.

## 0. Compilar

Desde la raiz del repositorio:

```bash
cd tp_final_ws
colcon build --packages-select tp_platform tp_interfaces tp_a_slam_aruco \
  tp_b_navigation tp_c_mission turtlebot3_custom_simulation
source install/setup.bash
cd ..
```

## 1. Ver contrato del bag

```bash
bash docs/parte_a/scripts/laboratorio_bag.sh info
```

Esperado:

- odometria: `/tb4_1/odom`;
- LIDAR: `/tb4_1/scan`;
- imagen RGB: `/tb4_1/oakd/rgb/preview/image_raw`;
- camera info: `/tb4_1/oakd/rgb/preview/camera_info`;
- TF: `/tb4_1/tf` y `/tb4_1/tf_static`.

## 2. Primera pasada: Graph SLAM

Terminal 1:

```bash
bash docs/parte_a/scripts/laboratorio_bag.sh slam
```

Terminal 2:

```bash
bash docs/parte_a/scripts/laboratorio_bag.sh play
```

Cuando termina el bag, cortar Terminal 1 con `Ctrl+C`. Debe quedar:

```text
runs/laboratorio/parte_a/trajectory.json
runs/laboratorio/parte_a/aruco_detections.csv
runs/laboratorio/parte_a/aruco_geometry_debug.csv
```

Chequeo rapido:

```bash
test -s runs/laboratorio/parte_a/trajectory.json
python3 - <<'PY'
import json
from pathlib import Path
p = Path("runs/laboratorio/parte_a/trajectory.json")
d = json.loads(p.read_text())
print("poses:", len(d.get("trajectory", [])))
print("landmarks:", len(d.get("landmarks", {})))
PY
```

Si hay muy pocas poses o landmarks, no conviene generar mapa todavia: primero
revisar detecciones ArUco, frame de camara y TF.

## 3. Segunda pasada: mapa

Terminal 1:

```bash
bash docs/parte_a/scripts/laboratorio_bag.sh map
```

Terminal 2:

```bash
bash docs/parte_a/scripts/laboratorio_bag.sh play
```

Cuando termina el bag, cortar Terminal 1 con `Ctrl+C`. Debe quedar:

```text
runs/laboratorio/parte_a/map.yaml
runs/laboratorio/parte_a/map.pgm
```

## 4. Probar localizacion offline

Terminal 1:

```bash
bash docs/parte_a/scripts/laboratorio_bag.sh localize
```

Terminal 2:

```bash
bash docs/parte_a/scripts/laboratorio_bag.sh play
```

En RViz:

1. usar `Fixed Frame: map`;
2. publicar una pose inicial aproximada con `2D Pose Estimate`;
3. mirar `/particlecloud`, `/mcl_pose`, `/map`, `/scan` y TF.

El launch remapea la velocidad a `/test/cmd_vel`, asi que esta prueba offline no
comanda ningun robot.

## 5. Criterio de decision

Mapa usable:

- paredes principales continuas;
- puertas o pasillos transitables no cerrados;
- sin paredes dobles severas;
- sin conexiones falsas grandes entre habitaciones/pasillos.

Localizacion usable:

- la nube de particulas converge cerca de la pose inicial;
- `/mcl_pose` se mantiene sobre el corredor correcto al reproducir el bag;
- las correcciones ArUco no producen saltos grandes e incoherentes;
- `map -> odom` no oscila violentamente.

Si el mapa sale mal pero el SLAM tiene landmarks razonables, revisar la segunda
pasada de LIDAR. Si el mapa sale bien pero MCL no converge, revisar pose inicial,
landmarks cargados desde `trajectory.json` y remapeos `tb4_1`.
