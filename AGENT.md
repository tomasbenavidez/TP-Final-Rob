# AGENT.md — Contexto de sesión: TP Final Robótica Autónoma (Parte A)

## Situación actual

**RESUELTO (2026-06-16):** la reconstrucción del mapa salía irreconocible ("blob"). Tras un
diagnóstico con método científico (hipótesis + experimentos de reproyección offline) se
identificaron y corrigieron **dos causas raíz**. Ver la sección "Diagnóstico del mapa
(resuelto)" más abajo. El mapa reconstruido ahora es un laberinto limpio y reconocible
(~4700 celdas-pared vs. ~12400 antes).

---

## Qué es este proyecto

**Graph SLAM con ArUco** para el TurtleBot4. Usa un RosBag real del laberinto de la cátedra.

Pipeline en dos pasadas:
1. **1ª pasada** (`parte_a_slam.launch.py`): detecta marcadores ArUco con la cámara, construye
   un grafo de poses en GTSAM (odometría + observaciones ArUco), optimiza con
   Levenberg-Marquardt, y guarda la trayectoria corregida en `/tmp/trayectoria.json`.
2. **2ª pasada** (`parte_a_mapa.launch.py`): re-reproduce el bag y proyecta los scans LIDAR
   sobre la trayectoria ya corregida usando un inverse sensor model con log-odds + Bresenham.
   Exporta `mapa.pgm` + `mapa.yaml` (formato nav2 map_server).

Materia: I-402 Principios de la Robótica Autónoma, UdeSA.
Opción elegida: Opción 3 (Features con cámara, Graph SLAM obligatorio).

---

## Árbol del repo

```
TP-Final-Rob/
├── AGENT.md                         ← este archivo
├── README.md                        ← instrucciones de uso completas
├── resources/
│   ├── PRA_TPFinal_Parte_A (1).md   ← enunciado del TP
│   ├── 16-graph-slam-3 (1).md       ← slides de clase: Graph SLAM
│   └── grisetti10titsmag.md         ← tutorial Grisetti et al. (referencia teórica)
└── tp_final_ws/
    ├── bags/
    │   ├── aruco_estimation/        ← bag corto: ArUco a distancias controladas
    │   └── laberinto/               ← bag largo: recorrido completo del laberinto
    └── src/tp_slam_aruco/
        ├── config/
        │   ├── camera_tb4_0.yaml    ← calibración intrínseca OAK-D RGB
        │   └── rviz_config.rviz
        ├── launch/
        │   ├── parte_a_slam.launch.py   ← 1ª pasada
        │   └── parte_a_mapa.launch.py   ← 2ª pasada
        ├── test/
        └── tp_slam_aruco/           ← código Python
```

---

## Módulos clave y sus roles

| Archivo | Rol |
|---|---|
| `graph_slam_node.py` | Nodo central. Suscribe odometría + detecciones ArUco, construye grafo GTSAM, optimiza, publica TF `map→odom` y trayectoria |
| `aruco_detector_node.py` | Detecta ArUco en cada frame de cámara, estima pose 3D con solvePnP, filtra y publica como MarkerArray |
| `occupancy_grid_node.py` | 2ª pasada: lee trayectoria JSON, proyecta scans LIDAR en grilla log-odds, exporta PGM+YAML |
| `slam_graph.py` | Wrapper de GTSAM: `optimize_graph()` con LM + rebuild de `initial` desde result |
| `slam_gating.py` | Validación de re-observaciones: Mahalanobis (chi² 2-DOF) + spatial jump |
| `slam_geometry.py` | Transformaciones cámara→base y predicción de landmark en world frame |
| `slam_landmarks.py` | Gate de promoción: acepta landmark nuevo solo tras N keyframes (default 3) |
| `slam_mapping.py` | Bresenham, interpolación de poses, log-odds→occupancy |
| `motion_model.py` | Modelo δrot1/δtrans/δrot2 (usado por `odometry_node`), `normalize_angle`, `yaw_from_quaternion` |
| `aruco_utils.py` | `estimate_marker_poses` con SOLVEPNP_IPPE_SQUARE, carga YAML de calibración |
| `aruco_filtering.py` | Filtros de detección: área, profundidad, error de reproyección, whitelist IDs |
| `tf_bridge_node.py` | Republica `/tb4_0/tf` y `/tb4_0/tf_static` del bag a `/tf` y `/tf_static` |
| `slam_io.py` | Serialización JSON de la trayectoria (read/write) |
| `slam_debug.py` | CSV de diagnóstico geométrico: residuales, fuentes TF, jumps espaciales |
| `slam_publish.py` | Constructores de mensajes ROS para RViz (PoseArray, Path, MarkerArray, TF) |

---

## Diseño del Graph SLAM (lo que hay que saber para depurar)

### Factores del grafo
- `PriorFactorPose2(X(0), Pose2(0,0,0))` — ancla el grafo en el origen
- `BetweenFactorPose2(X(i-1), X(i), relative_pose)` — odometría: usa `Pose2.between()` (SE(2) exacto)
- `BearingRangeFactor2D(X(i), L(id), bearing, range)` — observación ArUco desde keyframe i

### Ruido de observación (caracterizado con `aruco_estimation`)
```python
sigma_bearing = 0.10 + 0.05 * range
sigma_range   = 0.0239 + 0.0315 * range²
```
Envuelto en kernel **Cauchy** (`cauchy_k=1.0`) para robustez ante outliers.

### Creación de keyframes
Trigger: `dist >= 0.15 m` OR `|Δθ| >= 0.60 rad` desde el último KF.
Las detecciones ArUco pendientes entre KFs se adjuntan al KF más reciente.
Solo se guarda la detección más cercana por ID (menor ruido).

### Loop closure (pipeline de gating)
1. **Parallax gate**: `‖pos_actual − pos_ultima_obs‖ >= 0.20 m`
2. **Spatial gate**: posición predicha del landmark vs. posición guardada, umbral 0.75 m
3. **Innovation gate**: Mahalanobis² en espacio (bearing, range), umbral 5.99 (chi² 95%)

### Promoción de landmarks
Un ID nuevo se acepta al grafo solo después de aparecer en **3 keyframes distintos**
(`min_landmark_observations=3`). Evita landmarks de detecciones espurias.

### TF `map → odom`
```python
T_map_odom = T_map_base · (T_odom_base)⁻¹
```
Publicado a 20 Hz por un timer con `STEADY_TIME` (funciona aunque `use_sim_time=true`).

### Extrínsecos cámara→base
Prioridad: **TF lookup real** del bag > fallback por parámetro.
Fallback TB4: `tx=-0.0596 m`, `ty=0.0`, `yaw=0.0`.

---

## Cómo correr el sistema

### Build
```bash
cd tp_final_ws
colcon build --packages-select tp_slam_aruco
source install/setup.bash
```

### 1ª pasada (SLAM)
```bash
# Terminal 1
ros2 bag play tp_final_ws/bags/laberinto --clock

# Terminal 2
ros2 launch tp_slam_aruco parte_a_slam.launch.py \
  calibration_file:=tp_final_ws/src/tp_slam_aruco/config/camera_tb4_0.yaml \
  trajectory_file:=/tmp/trayectoria.json \
  use_bag_tf:=true
# Ctrl+C al terminar → guarda /tmp/trayectoria.json
```

### 2ª pasada (mapa)
```bash
# Terminal 1
ros2 bag play tp_final_ws/bags/laberinto --clock

# Terminal 2
ros2 launch tp_slam_aruco parte_a_mapa.launch.py \
  trajectory_file:=/tmp/trayectoria.json \
  map_output:=/tmp/mapa
# Ctrl+C → guarda /tmp/mapa.pgm + /tmp/mapa.yaml
```

### Diagnóstico rápido post-1ª pasada
```bash
# Resumen de detecciones
python3 - <<'PY'
import csv
from collections import Counter
rows = list(csv.DictReader(open('/tmp/aruco_detections.csv')))
accepted = [r for r in rows if r['accepted'] == '1']
print('raw:', len(rows), 'aceptados:', len(accepted))
print('ids aceptados:', sorted(Counter(r['id'] for r in accepted).items()))
print('rechazos:', Counter(r['reason'] for r in rows if r['accepted'] == '0'))
PY

# Residuales de geometría
python3 - <<'PY'
import csv, statistics
rows = [r for r in csv.DictReader(open('/tmp/aruco_geometry_debug.csv')) if r['residual_range']]
res = [abs(float(r['residual_range'])) for r in rows]
print('filas con residual:', len(rows))
if res:
    print('min/med/max:', min(res), statistics.median(res), max(res))
    print('residual > 0.4 m:', sum(1 for v in res if v > 0.4))
PY
```

---

## Parámetros ajustables clave

| Parámetro | Default | Dónde ajustar |
|---|---|---|
| `kf_dist` | 0.15 m | launch arg |
| `kf_angle_max` | 0.60 rad | launch arg |
| `min_landmark_observations` | 3 | launch arg |
| `max_landmark_position_jump` | 0.75 m | launch arg |
| `reobs_min_parallax` | 0.20 m | launch arg |
| `maha_threshold` | 5.99 | `graph_slam_node.py:63` (no expuesto aún en launch) |
| `cauchy_k` | 1.0 | `graph_slam_node.py:64` |
| `map_margin` | 3.0 m | launch arg (nuevo, auto-sizing grilla) |
| `allowed_marker_ids` | '' (todos) | launch arg |
| `MAX_OBS_PER_LANDMARK` | 50 | `graph_slam_node.py` (constante de clase) |
| `lidar_yaw` (2ª pasada) | π/2 | launch `parte_a_mapa` (montaje real +90° del RPLIDAR) |
| `max_angular_velocity` (2ª pasada) | 0.0 (off) | launch `parte_a_mapa` (gate "no mapear girando") |

---

## Cambios aplicados en esta sesión (rama `parte_A_toto`)

1. **`_rvec_to_quaternion`** → reemplazado por scipy:
   ```python
   R, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float64))
   q = Rotation.from_matrix(R).as_quat()
   ```

2. **Thresholds PGM/YAML consistentes** (`occupancy_grid_node.py`):
   - Antes: código usaba 0.6/0.4, YAML decía 0.65/0.196 → confuso e inconsistente
   - Ahora: constantes de clase `_PGM_OCC_THRESH=0.60`, `_PGM_FREE_THRESH=0.40`,
     `_PGM_UNKNOWN_PX=127`. El pixel desconocido es 127 (prob≈0.5, cae entre 0.40 y 0.60).
     El YAML se escribe con las mismas constantes → siempre consistentes.

3. **Auto-sizing de la grilla** (`occupancy_grid_node.py`):
   - Si hay trayectoria cargada, calcula `origin_x/y`, `width`, `height` desde el bounding box
     de las poses + `map_margin=3.0 m` por lado.
   - Los parámetros `width/height/origin_x/origin_y` quedan como fallback sin trayectoria.

4. **`motion_model.py` no se tocó**: está activamente usado por `odometry_node.py`
   (`compute_delta`, `apply_delta`, `OdometryDelta`) y por `graph_slam_node.py` +
   `slam_gating.py` (`normalize_angle`, `yaw_from_quaternion`).

---

## Diagnóstico del mapa (resuelto)

El mapa salía como un "blob" irreconocible. Diagnóstico con método científico (reproyección
offline de los scans del bag sobre distintas trayectorias, contando celdas-pared como métrica
de nitidez). Resultado: **dos causas raíz independientes**, ambas en la 2ª pasada, **NO en el
Graph SLAM** (cuyo heading coincide con la odometría a 0.4° de mediana).

### Causa raíz #1 — LIDAR mal orientado (yaw)

El TF estático del bag dice:
```
base_link -> shell_link    (yaw 0)
shell_link -> rplidar_link  t=(-0.04, 0, +0.0987)  yaw = +90° (+π/2)
```
El RPLIDAR está montado **rotado +90°** respecto de `base_link`. `occupancy_grid_node` usaba
`lidar_yaw=0` → cada barrido salía rotado y, como el robot gira a lo largo del recorrido, una
misma pared observada desde distintos rumbos caía en celdas distintas → smearing.
**Fix:** `lidar_yaw = π/2` (default del nodo y del launch de la 2ª pasada). `lidar_tx=-0.04` ya
estaba bien.

### Causa raíz #2 (dominante) — trayectoria sparse + interpolación lineal

El Graph SLAM entrega una trayectoria **correcta pero sparse** (280 keyframes, gaps de hasta
11.8 s). La 2ª pasada interpolaba linealmente esos keyframes para datar cada scan; durante los
giros (rotación no uniforme entre keyframes) eso falsea la pose y emborrona las paredes.
Experimento decisivo: submuestrear la odometría densa a los 280 timestamps de keyframe y
re-interpolar reproduce el mismo blob (12473 celdas) que el SLAM; la odom densa da un mapa
limpio (4907).
**Fix (enfoque de pose-graph mapping estándar):** la 2ª pasada compone
```
pose_map(t) = corrección_SLAM_interpolada(t) ∘ odom_densa(t)
```
La corrección SLAM (`T_map_odom`, lenta y suave) se aplica sobre la odometría densa (20 Hz, alto
detalle). Para esto la 1ª pasada ahora guarda la **pose de odom cruda de cada keyframe** en el
JSON (`pose['odom']`), y `occupancy_grid_node` se suscribe a `/tb4_0/odom` y bufferea la odom
densa. Si el JSON no trae `odom` (formato viejo) cae a interpolación lineal (modo legacy).
Resultado: ~4726 celdas-pared, laberinto reconocible.

### Pedidos del usuario (incorporados)

- **No mapear mientras gira**: parámetro `max_angular_velocity` (rad/s) en la 2ª pasada; si
  `|ω|` de la odom densa supera el umbral, el scan no se integra. **Default 0.0 (desactivado)**
  porque en este bag el robot gira lento (ω máx 0.36 rad/s → smear de barrido < 3°) y el enfoque
  de odom densa ya elimina el smearing de giro. Activar con `max_angular_velocity:=0.3` si se
  desea (descarta solo el ~0.2% de scans más rápidos).
- **Más de 1 landmark en el mapeo**: verificado. El grafo usa **46 landmarks** (mediana 5 obs
  c/u, máx 13), **41 con ≥3 observaciones** (loop closures sólidos) y **58 keyframes ven ≥2
  landmarks simultáneos**. El mapeo LIDAR en sí no usa landmarks (solo la trayectoria); los
  landmarks restringen la optimización del SLAM, que está bien constreñida.

### Nota: error GLSL en RViz al poner Durability=Transient Local

`active samplers with a different type refer to the same texture image unit` es un bug de
linkeo de shaders de OGRE/RViz2 bajo render por software (WSL2/llvmpipe), **no** del pipeline ni
del mapa. El mapa igual se publica/exporta bien. Workarounds: dejar Durability en `Volatile` (y
republicar), o exportar el PGM y verlo con `map_server` / visor de imágenes, o forzar
`export LIBGL_ALWAYS_SOFTWARE=1` antes de lanzar RViz.

### Métricas de diagnóstico (siguen siendo útiles)

Después de la 1ª pasada: landmarks inicializados, loop closures, error del grafo, proporción de
detecciones que llegan al grafo (`logs_runs/aruco_detections.csv`,
`logs_runs/aruco_geometry_debug.csv`). Un sistema sano: varios landmarks, múltiples loop
closures, error del grafo decayendo tras los cierres.

---

## Dependencias Python

```bash
pip install "numpy<2" gtsam pyyaml opencv-contrib-python scipy
python3 -c "import cv_bridge, cv2, gtsam, numpy, scipy; print(numpy.__version__)"
```

## Archivos de diagnóstico generados en runtime

| Archivo | Contenido |
|---|---|
| `/home/tomasbenavidez/Robotica/TP-Final-Rob/tp_final_ws/logs_runs/trayectoria.json` | Poses optimizadas + landmarks (salida de la 1ª pasada) |
| `/home/tomasbenavidez/Robotica/TP-Final-Rob/tp_final_ws/logs_runs/aruco_detections.csv` | Cada detección ArUco: id, accepted, reason, tvec, area, reproj |
| `/home/tomasbenavidez/Robotica/TP-Final-Rob/tp_final_ws/logs_runs/aruco_geometry_debug.csv` | Observaciones del grafo: tf_source, x_base, y_base, range, residual, spatial_jump |
| `/home/tomasbenavidez/Robotica/TP-Final-Rob/tp_final_ws/logs_runs/mapa.pgm` + `/home/tomasbenavidez/Robotica/TP-Final-Rob/tp_final_ws/logs_runs/mapa.yaml` | Mapa final (salida de la 2ª pasada) |
