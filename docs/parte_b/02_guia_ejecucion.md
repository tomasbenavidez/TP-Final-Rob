# Parte B — Guía de ejecución (correr todo + sacar screenshots)

Instrucciones para levantar el ciclo completo de Parte B, qué se ve en cada paso, qué comandos
usar en **Gazebo** y **RViz**, y qué screenshots faltan. Pensado para correr en el Mac del equipo
(ROS 2 Humble por RoboStack, env conda `rosenv`).

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
> En una PC Linux normal alcanzaría con `source install/setup.zsh` (o `.bash`) y los `ros2 launch`
> de siempre.

---

## 0. Compilar (una vez)

```bash
source ~/Documents/GitHub/TP-Final-Rob/docs/parte_b/scripts/setup_parte_b.sh
cd ~/Documents/GitHub/TP-Final-Rob/tp_final_ws
colcon build --packages-select tp_b_navigation
```

---

## 1. Levantar la simulación de Gazebo (Terminal 1)

```bash
source ~/Documents/GitHub/TP-Final-Rob/docs/parte_b/scripts/setup_parte_b.sh
# entorno estándar:
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
# con obstáculos no mapeados (para probar la evasión, consigna 1.9):
# ros2 launch turtlebot3_custom_simulation custom_casa_obs.launch.py
```

**Qué se ve en Gazebo:** la casa (el mismo entorno cuyo mapa generó la Parte A) con el TurtleBot3
*burger* spawneado en el origen `(0,0)`. Publica `/scan` (LIDAR), `/odom`, `/clock` y la TF
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
source ~/Documents/GitHub/TP-Final-Rob/docs/parte_b/scripts/setup_parte_b.sh
ros2 launch tp_b_navigation parte_b.launch.py
```

Esto levanta los 7 nodos (map_loader, landmark_publisher, landmark_sensor, mcl_localization,
global_planner, obstacle_monitor, state_machine) **+ RViz** con la config de Parte B, todos con
`use_sim_time:=true`.

Argumentos útiles:
- `launch_rviz:=false` — no abrir RViz (si querés abrirlo aparte).
- `map_yaml:=/ruta/otro_map.yaml` — usar otro mapa.

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
| Landmarks GT | `/landmarks_markers` | 36 estrellas/cilindros verdes sobre las paredes |
| Observed Landmarks | `/observed_landmarks_markers` | puntos naranjas: lo que la "cámara" ve este frame |
| LaserScan | `/scan` | el LIDAR (best_effort) |
| MCL Particles | `/particlecloud` | la nube de partículas del filtro |
| Path | `/plan` | la ruta A* al goal |
| TF | — | la cadena `map→odom→base_footprint` |

Para ver el estado de la FSM en consola: `ros2 topic echo /nav_state`.

---

## 4. Verificación rápida sin RViz (por si el GUI no abre)

```bash
source ~/.../scripts/setup_parte_b.sh
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
  (su *MessageFilter*) → **parpadea pero los markers igual aparecen**. No pierde dato ni afecta al
  MCL/navegación (el MCL asocia por índice, no transforma). **Ya está mitigado en el código:**
  (1) el MCL publica `map→odom` con el stamp adelantado `transform_tolerance=0.1 s` (truco de AMCL),
  y (2) el `landmark_sensor` estampa `/observed_landmarks` con el tiempo de la TF que realmente usó,
  no con el stamp futuro del scan. Con eso la transformación pasa ~100% al primer intento. Si
  reaparece, subí `transform_tolerance`.
- **Los nodos no reciben `/scan` ni `/odom` (suscriptos pero 0 datos):** es el DDS. El setup ya
  pone `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + `CYCLONEDDS_URI` apuntando a
  `scripts/cyclonedds_loopback.xml` (loopback unicast, `MaxAutoParticipantIndex=100`). **Todas las
  terminales** deben tener el mismo `source`.
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
