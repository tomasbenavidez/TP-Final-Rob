# Parte B — Guía de ejecución (correr todo + sacar screenshots)

Instrucciones para levantar el ciclo completo de Parte B, qué se ve en cada paso, qué comandos
usar en **Gazebo** y **RViz**, y qué screenshots faltan. Los paquetes usan los mismos launch
en ROS 2 Humble sobre Linux y sobre macOS/RoboStack.

Salvo que se indique lo contrario, los comandos parten de la raíz del repositorio clonado.

> **Importante (macOS):** preparar el entorno. Dos opciones:
> - **Recomendada (lo que ya usás):** en tu terminal interactiva normal,
>   `source install/setup.zsh` **funciona** (los warnings de `compdef` son inofensivos). Sólo
>   asegurate de exportar el DDS en loopback (FastDDS no descubre entre procesos en este Mac):
>   `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` y
>   `export CYCLONEDDS_URI=file://$PWD/docs/parte_b/scripts/cyclonedds_loopback.xml`.
> - **Alternativa todo-en-uno:** `source docs/parte_b/scripts/setup_parte_b.sh` (arma el entorno
>   y fija el DDS de una). Hace falta sí o sí si corrés en un zsh **no-interactivo** (scripts,
>   tareas automáticas), donde `setup.zsh` aborta por los errores de `compdef`.
>
> En Linux alcanza con `source tp_final_ws/install/setup.bash`. No hace falta Conda ni configurar
> un RPATH manual.

---

## 0. Compilar (una vez)

Linux:

```bash
cd tp_final_ws
colcon build --packages-select \
  tp_platform tp_interfaces tp_a_slam_aruco tp_b_navigation \
  tp_c_mission turtlebot3_custom_simulation
source install/setup.bash
```

macOS/RoboStack:

```zsh
rosenv
cd tp_final_ws
colcon build --packages-select \
  tp_platform tp_interfaces tp_a_slam_aruco tp_b_navigation \
  tp_c_mission turtlebot3_custom_simulation
source install/setup.zsh
```

En el Mac con RoboStack, el entorno `rosenv_mf` usa Python 3.11, NumPy 1.26.4 y
pytest 7.4.4. Esta versión de pytest es necesaria porque el `launch_testing 1.0.4`
de ROS 2 Humble no es compatible con pytest 9.

Para reconstruir `tp_interfaces` desde cero, abrí una terminal nueva, ejecutá
`rosenv` y borrá sus directorios **antes** de sourcear `install/setup.zsh`:

```bash
cd tp_final_ws
rm -rf build/tp_interfaces install/tp_interfaces
colcon build --packages-select \
  tp_platform tp_interfaces tp_a_slam_aruco tp_b_navigation \
  tp_c_mission turtlebot3_custom_simulation \
  --cmake-args -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python3"
source install/setup.zsh
```

Si se borra un paquete después de sourcear el workspace en la misma terminal,
`AMENT_PREFIX_PATH` y `CMAKE_PREFIX_PATH` conservan temporalmente la ruta eliminada
y `colcon` avisa que no existe. Una terminal nueva evita esos warnings sin editar
manualmente las variables de entorno.

Si se actualizó el enlazado de `turtlebot3_custom_simulation`, forzá una
reconfiguración portable sin borrar el resto del workspace:

```bash
colcon build --packages-select turtlebot3_custom_simulation --cmake-clean-cache
```

---

## 1. Levantar la simulación de Gazebo (Terminal 1)

```bash
source docs/parte_b/scripts/setup_parte_b.sh
# entorno estándar:
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
# con obstáculos no mapeados (para probar la evasión, consigna 1.9):
# ros2 launch turtlebot3_custom_simulation custom_casa_obs.launch.py
```

**Qué se ve en Gazebo:** la casa (el mismo entorno cuyo mapa generó la Parte A) con el TurtleBot3
*burger* spawneado en el origen `(0,0)`. Publica `/scan` (LIDAR), `/odom` como verdad
simulada, `/calc_odom` para la predicción, `/clock` y la TF
`odom→base_footprint`.

> En `custom_casa_obs` aparecen además obstáculos (valijas/sofá/mesa) que **no están en el mapa**.

> **Ojo con instancias zombi.** Si dejaste un Gazebo viejo corriendo, una nueva `ros2 launch`
> puede mostrarte la instancia vieja (mismo dominio DDS) y parecer que el robot está trabado /
> no se mueve. Si el robot "no responde a ningún goal", casi seguro hay un gazebo previo:
> `pkill -9 -f "gzserver|gzclient|tp_b_navigation"` y volvé a lanzar. El spawn en `(0,0)` está
> libre (la mesa más cercana está a ~0.83 m); si querés más margen, spawneá en otro lado con
> `x_pose:=` / `y_pose:=` (ej. `ros2 launch ... custom_casa.launch.py x_pose:=0.0 y_pose:=-1.5`).

---

## 2. Levantar la pila de Parte B (Terminal 2)

```bash
source docs/parte_b/scripts/setup_parte_b.sh
ros2 launch tp_b_navigation parte_b.launch.py
```

Esto levanta los 7 nodos (map_loader, landmark_publisher, landmark_sensor, mcl_localization,
global_planner, obstacle_monitor, state_machine) **+ RViz** con la config de Parte B, todos con
`use_sim_time:=true`.

Argumentos útiles:
- `launch_rviz:=false` — no abrir RViz (si querés abrirlo aparte).
- `map_yaml:=/ruta/otro_map.yaml` — usar otro mapa.

### Perfil real TB4

Para ejecutar Parte B sobre el mapa aceptado de Parte A:

```bash
source tp_final_ws/install/setup.bash
ros2 launch tp_b_navigation parte_b.launch.py \
  profile:=real_tb4 \
  robot_namespace:=tb4_0 \
  map_yaml:=/tmp/tb4-run/map/map.yaml \
  landmark_map_file:=/tmp/tb4-run/parte_a/trayectoria.json
```

El perfil real usa `/tb4_0/odom`, `/tb4_0/scan`, `/tb4_0/cmd_vel` y la cámara
OAK-D derivadas de `robot_namespace`. Agrega `aruco_detector` y
`aruco_mcl_adapter` para alimentar `/observed_landmark_ids` al MCL. No inicia
`landmark_publisher`, `landmark_sensor`, `/calc_odom`, drivers TB4 ni Graph SLAM.
El MCL no hace SLAM: usa el mapa ya cargado en `/map`, la odometría, los ArUco
identificados y el LIDAR contra la grilla para estimar `map→odom`.

Para una prueba pasiva sobre bag, usá el launch pasivo: no inicia planner,
`state_machine`, `obstacle_monitor` ni ningún productor de `/cmd_vel`.

```bash
ros2 launch tp_b_navigation parte_b_bag_localization.launch.py \
  profile:=bag_tb4 \
  map_yaml:=/tmp/tb4-run/map/map.yaml \
  landmark_map_file:=/tmp/tb4-run/parte_a/trayectoria.json \
  diagnostics_csv:=/tmp/tp_mcl_laberinto.csv \
  compensation_diagnostics_csv:=/tmp/aruco_mcl_compensation.csv
```

Para probar la corrección fuera de secuencia de ArUco, activá OOS en MCL. En
ese modo el adapter deja de compensar el atraso y publica el timestamp original
de la imagen; MCL corrige una nube histórica en `t_obs`, la repropaga con
odometría hasta `now` y reemplaza la nube actual:

```bash
ros2 launch tp_b_navigation parte_b_bag_localization.launch.py \
  profile:=bag_tb4 \
  map_yaml:=/tmp/tb4-run/map/map.yaml \
  landmark_map_file:=/tmp/tb4-run/parte_a/trayectoria.json \
  laser_log_weight:=0.25 \
  use_oos_landmark_updates:=true \
  diagnostics_csv:=/tmp/tp_mcl_laberinto_oos.csv
```

En otra terminal reproducí el bag con clock:

```bash
ros2 bag play tp_final_ws/bags/laberinto --clock
```

Después publicá una pose inicial aproximada desde RViz o por CLI:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

Si `map_yaml` o `landmark_map_file` quedan en defaults flexibles, el launch avisa
por consola. En `real_tb4` quedan activos los safety gates:
`enable_safety_gates`, `max_mcl_pose_age`, `max_scan_age`,
`max_position_covariance` y `max_yaw_covariance`. Estos gates frenan movimiento
autónomo e inserción de obstáculos dinámicos si la pose MCL, la covarianza, la TF
o el scan no son confiables.

El CSV de diagnóstico es opcional. Cuando `diagnostics_csv` no está vacío, MCL
agrega una fila por corrección de medición (`event=laser`, `event=landmark` o
`event=landmark_oos`).
Las columnas más útiles para diagnosticar saltos son `delta_xy`, `delta_yaw`,
`n_eff_before`, `n_eff_after`, `resampled`, `used` y `detail`. Si un salto grande
aparece en una fila `laser`, el likelihood del LIDAR está dominando; si aparece
en una fila `landmark` o `landmark_oos`, revisar asociación/TF/calibración de
ArUco. En OOS, `detail` agrega `age_sec`, `snapshot_dt_sec` y `replay_steps`.

En TB4 real/bag, `aruco_mcl_adapter` compensa observaciones visuales retardadas
usando la odometría relativa entre el timestamp de la imagen y el tiempo actual.
La medición conserva su `observation.header.stamp` original, pero el array
`/observed_landmark_ids` se publica con stamp actual para que MCL la trate como
una observación compensada. El archivo `compensation_diagnostics_csv` permite
auditar `age_sec`, `compensated`, `drop_reason` y el movimiento relativo usado.
Comparar junto con `/tmp/aruco_image_timing.csv` ayuda a distinguir delay de
cámara, compensación aplicada y saltos finales de MCL.

---

## 3. Operar desde RViz

La config `parte_b.rviz` ya trae los displays. Fixed frame = `map`.

1. **"2D Pose Estimate"** (barra de herramientas) → clic+arrastre sobre la pose real del robot.
   Publica en `/initialpose`. El MCL siembra las partículas ahí.
   - *Se ve:* la nube de partículas (`/particlecloud`) aparece alrededor del clic y se concentra.
2. **"2D Goal Pose"** → clic+arrastre en el destino (la flecha marca el ángulo final).
   Publica en `/goal_pose`.
   - *Se ve:* aparece la ruta `/plan` (línea), el robot empieza a moverse y la sigue.
3. El robot llega a la posición, **gira al ángulo final** y queda en `GOAL_REACHED`. Podés mandar
   otro goal cuando quieras (re-planea). Si aparece un obstáculo no mapeado, entra en `AVOIDING` y
   re-planea.

### Displays de RViz y qué muestran
| Display | Topic | Qué se ve |
|---|---|---|
| Map | `/map` | la grilla de ocupación de la casa |
| Landmarks GT | `/landmarks_markers` | 60 estrellas/cilindros verdes sobre las paredes |
| Observed Landmarks | `/observed_landmarks_markers` | puntos naranjas: lo que la "cámara" ve este frame |
| Observed Landmark IDs | `/observed_landmark_ids` | mediciones range/bearing con ID explícito para el MCL |
| LaserScan | `/scan` | el LIDAR (best_effort) |
| MCL Particles | `/particlecloud` | la nube de partículas del filtro |
| Path | `/plan` | la ruta A* al goal |
| TF | — | la cadena `map→odom→base_footprint` |

Para ver el estado de la FSM en consola: `ros2 topic echo /nav_state`.

---

## 4. Verificación rápida sin RViz (por si el GUI no abre)

```bash
source docs/parte_b/scripts/setup_parte_b.sh
ros2 topic echo /nav_state                       # estado de la FSM
ros2 run tf2_ros tf2_echo map base_footprint     # pose estimada por el MCL
ros2 topic echo --once /plan nav_msgs/msg/Path   # la ruta planeada
```
Para mandar pose/goal por línea de comando (equivale a los clics de RViz):
```bash
# 2D Pose Estimate en (0,0,0):
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
# 2D Goal Pose en (-2.0, 2.2) mirando a +y:
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: -2.0, y: 2.2}, orientation: {z: 0.707, w: 0.707}}}"
```

---

## 5. Troubleshooting (lo que aprendimos en este Mac)

- **`source install/setup.bash` no anda / "package not found":** en zsh interactivo usá
  `source install/setup.zsh` (los warnings de `compdef` son inofensivos). En zsh **no-interactivo**
  (scripts) usá `source docs/parte_b/scripts/setup_parte_b.sh`, que arma todo a mano.
- **RViz tira y saca un error en loop sobre `observed_landmarks`:** *"No transform to fixed frame
  [map]... Lookup would require extrapolation into the future. Requested time X but the latest data
  is at Y"*. Es un artefacto de **sim-time**: el plugin del LIDAR de Gazebo estampa el `/scan`
  ~20-60 ms **adelante** de su propia TF `odom→base`, y `/clock` llega al suscriptor con un poco de
  lag → el mensaje queda "en el futuro" para la TF por unos ms, RViz lo descarta y reintenta
  (su *MessageFilter*) → **parpadea pero los markers igual aparecen**. La corrección por landmarks
  del MCL no transforma esos markers en RViz; la corrección láser usa `/scan` y TF aparte.
  **Ya está mitigado en el código:**
  (1) el MCL publica `map→odom` con el stamp adelantado `transform_tolerance=0.1 s` (truco de AMCL),
  y (2) el `landmark_sensor` estampa `/observed_landmarks` con el tiempo de la TF que realmente usó,
  no con el stamp futuro del scan. Con eso la transformación pasa ~100% al primer intento. Si
  reaparece, subí `transform_tolerance`.
- **Los nodos no reciben `/scan` ni `/odom` (suscriptos pero 0 datos):** es el DDS. El setup ya
  pone `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + `CYCLONEDDS_URI` apuntando a
  `scripts/cyclonedds_loopback.xml` (loopback unicast, `MaxAutoParticipantIndex=100`). **Todas las
  terminales** deben tener el mismo `source`.
- **`/calc_odom` no publica:** primero comprobá que el nodo calculador siga vivo:
  `ros2 node info /calculated_odometry` y luego
  `ros2 topic echo --once /calc_odom`. El ejecutable publica aun estando quieto.
  El smoke test automatizado se ejecuta, después del build y de sourcear el workspace, con
  `RUN_ROS_SMOKE=1 python3 -m pytest
  src/tp_b_navigation/test/test_calc_odom_smoke.py -q`.
  En macOS, `otool -l install/turtlebot3_custom_simulation/lib/turtlebot3_custom_simulation/turtlebot3_custom_simulation`
  debe mostrar un RPATH derivado del entorno ROS y nunca `/lib`; en Linux se puede inspeccionar
  el mismo binario con `ldd`.
- **No uses `ROS_LOCALHOST_ONLY=1`** con FastDDS: crashea gzserver (`foonathan::memory`).
- **Reiniciaste Gazebo:** reiniciá también la pila de Parte B. Al reiniciar Gazebo el reloj de
  simulación vuelve a 0 y los nodos vivos quedan con TF viejas (`TF_OLD_DATA`) → planean con poses
  stale.
- **No manejes el robot a mano mientras corre el `state_machine`:** los dos publican `/cmd_vel` y
  pelean. Mandá goals, o matá el `state_machine` si querés teleop.

---

## 6. Screenshots que faltan (para sacar con la compu)

> En el entorno donde se desarrolló, **gzclient y RViz arrancan y reciben datos pero no dibujan
> ventana** (contexto OpenGL 2.1 sin sesión gráfica al lanzarlos en background). **En una sesión
> normal de macOS (Terminal.app o iTerm, no la terminal integrada en background) deberían abrir
> bien.** Por eso estos screenshots quedan pendientes:

1. **Gazebo** — la casa con el robot (en `custom_casa` y en `custom_casa_obs` con los obstáculos).
2. **RViz — localización:** la nube de partículas concentrándose tras el "2D Pose Estimate", con
   los landmarks verdes y las observaciones naranjas.
3. **RViz — planificación:** la ruta `/plan` (línea) desde el robot hasta el goal, sobre el mapa.
4. **RViz — seguimiento:** el robot a mitad de camino siguiendo la ruta, con la TF y el `/scan`.
5. **RViz — llegada:** el robot en el goal, alineado al ángulo final.
6. **RViz — evasión:** en `custom_casa_obs`, el robot esquivando un obstáculo no mapeado
   (estado `AVOIDING` y la ruta re-planeada).

> en [`03_resultados_pruebas.md`](03_resultados_pruebas.md) hay **gráficos
> equivalentes generados con matplotlib** (mapa + landmarks + ruta + trayectoria real vs MCL) que
> muestran lo mismo sin depender del render de RViz.

---

## 7. Qué faltaría (estado al cierre)

- Sacar los screenshots de RViz/Gazebo del §6 (requiere la compu con GUI).
- Afinar el pico transitorio del MCL en traverses largos (ver `03_resultados_pruebas.md`).
- (Opcional, consigna 1.10) el mundo `custom_casa_obs2` **no está instalado** en este Mac; sólo hay
  `custom_casa` y `custom_casa_obs`.
- Integrar todo en el informe (entregable 2) y el video (entregable 3).
