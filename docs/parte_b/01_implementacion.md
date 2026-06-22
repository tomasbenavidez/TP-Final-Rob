# Parte B — Implementación (decisiones de diseño)

Documento de lo que **realmente se construyó** para la Parte B (navegación autónoma,
Sistema 3: grilla + landmarks de cámara virtuales). Complementa la arquitectura propuesta en
[`../context/04_arquitectura_parte_b.md`](../context/04_arquitectura_parte_b.md) con lo que
quedó implementado y por qué.

Todo el código nuevo está en el paquete **`tp_final_ws/src/tp_b_navigation`** (`ament_python`,
ROS 2 Humble). **No se tocó nada de Parte A.**

---

## 1. Mapa de nodos (lo construido)

```
                 /map (latched)
 map_loader ───────────────┬───────────────┬──────────────┐
                           │               │              │
 landmark_publisher ── /landmarks ──┐      │              │
                                    ▼       ▼              ▼
 /scan (Gazebo) ─────────► landmark_sensor  global_planner  obstacle_monitor
                                    │               ▲              │
                          /observed_landmarks       │ /plan_request │ /obstacle_detected
                                    ▼               │              ▼
 /odom (Gazebo) ─────────►   mcl_localization ──────┴──────► state_machine ──► /cmd_vel (Gazebo)
                                    │  TF map→odom                 │
                                    └──────────────────────────────┘ usa TF map→base
```

| Nodo | Ejecutable | Entradas | Salidas |
|---|---|---|---|
| `map_loader` | `map_loader` | `mapas/map.yaml` | `/map` (`OccupancyGrid`, latched) |
| `landmark_publisher` | `landmark_publisher` | param `landmarks` | `/landmarks` (`PoseArray`@map, latched), `/landmarks_markers` |
| `landmark_sensor` | `landmark_sensor` | `/scan`, `/landmarks`, TF | `/observed_landmarks` (`PoseArray`), markers |
| `mcl_localization` | `mcl_localization` | `/odom`, `/observed_landmarks`, `/initialpose` | `/mcl_pose`, `/particlecloud`, **TF `map→odom`** |
| `global_planner` | `global_planner` | `/map`, `/plan_request`, TF | `/plan` (`Path`), `/plan_status` |
| `obstacle_monitor` | `obstacle_monitor` | `/scan`, `/map`, TF | `/obstacle_detected` (`Bool`) |
| `state_machine` | `state_machine` | `/initialpose`, `/goal_pose`, `/plan`, `/obstacle_detected`, TF | `/cmd_vel`, `/nav_state` (`String`), `/plan_request` |

> **Decisión:** planner, monitor y máquina de estados quedaron **separados** (responsabilidades
> claras para el informe), pero el `state_machine` **fusiona** la FSM con el controlador de
> seguimiento (pure-pursuit), el ángulo final y la evasión — porque un único dueño de `/cmd_vel`
> es mucho más robusto que coordinar el control entre procesos. La arquitectura
> (`04_...md`) ya contemplaba esta fusión.

---

## 2. Cadena de TF

```
map ──(mcl_localization)──► odom ──(Gazebo)──► base_footprint ──► base_scan
```

- Gazebo publica `odom → base_footprint` (y `→ base_scan`).
- El **MCL** cierra la cadena publicando `map → odom` (la corrección de la odometría).
- Todos los nodos consumidores (planner, monitor, state_machine, sensor) leen la pose del
  robot con `lookup_transform('map', 'base_footprint')`.

---

## 3. Detalle por nodo

### 3.1 `map_loader` (consigna: cargar el mapa de Parte A)
- Parsea `map.yaml` + `map.pgm` (P5/P2) replicando la semántica **trinary** de `nav2_map_server`
  (`occupied_thresh`, `free_thresh`, `negate`) sin depender del stack de lifecycle de nav2.
- Publica `/map` con QoS **latcheado** (`transient_local`) + re-publicación periódica, para que
  RViz/planner/MCL que arranquen después igual lo reciban.
- Param: `map_yaml` (default instalado en `share/tp_b_navigation/maps/map.yaml`), `frame_id=map`.

### 3.2 `landmark_publisher` (Sistema 3: landmarks virtuales)
- Publica **36 landmarks fijos** en `/landmarks` (`PoseArray`@`map`, latched) + markers verdes.
- **Decisión de densidad:** se pasó de 14 esquinas a **36 puntos** sobre superficies de pared,
  distribuidos por *farthest-point sampling* del mapa (script `scripts/gen_landmarks.py` →
  `config/landmarks.yaml`). La consigna pide *"densidad coherente con el ArUco real"*; además,
  con pocos landmarks el MCL se volvía **multimodal** (ver §4).
- **Por qué sobre paredes:** el sensor virtual usa el LIDAR para la oclusión (§3.3); un landmark
  en espacio libre nunca devolvería un eco del LIDAR a su distancia → nunca sería "visible".

### 3.3 `landmark_sensor` (cámara virtual con oclusión)
Emula el detector ArUco que Gazebo no tiene. Por cada landmark conocido:
1. Lo proyecta al frame del robot y calcula **range + bearing**.
2. **FOV:** descarta los que caen fuera del rango angular/distancia del `/scan`.
3. **Oclusión (línea de visión):** compara el range esperado contra lo que mide el LIDAR en ese
   bearing; si el LIDAR ve algo más cerca (obstáculo interpuesto) → **ocluido**, no se publica.
4. Agrega **ruido gaussiano** (`sigma_range=0.05`, `sigma_bearing=0.05`).
- Salida `/observed_landmarks` (`PoseArray`): `position.x=range`, `position.z=bearing`,
  `(0,0,0)` si no visible. Asociación de datos **por índice** (orden = orden de `/landmarks`).
- **Stamp de la salida = tiempo de la TF que se usó, NO el stamp del `/scan`.** El LIDAR de Gazebo
  estampa el scan ~20-60 ms adelante de su propia TF `odom→base`; publicar con ese stamp dejaba la
  observación "en el futuro" para la TF y RViz parpadeaba. Estampar con `transform.header.stamp`
  garantiza que la cadena `map→odom→base` sea válida en ese instante.
- **Decisión crítica (`truth_frame`, default `odom`):** una cámara real ve los landmarks según la
  pose **verdadera** del robot, no según la estimación del filtro. Proyectar con la TF `map→robot`
  (la estimación del MCL) realimenta el filtro consigo mismo y lo hace **diverger**. Por eso el
  sensor usa `odom→base` (en Gazebo la odometría es ~ground-truth y `map≡odom` en el origen).

### 3.4 `mcl_localization` (filtro de partículas — consigna 1.4)
MCL con landmarks **conocidos y fijos** (a diferencia de FastSLAM del tp5, acá no se estiman).
- **Predicción:** por cada `/odom`, modelo de movimiento por odometría (δrot1, δtrans, δrot2 con
  ruido `alpha1..4`, Thrun cap. 5).
- **Corrección:** por cada `/observed_landmarks` visible, verosimilitud gaussiana range/bearing
  contra el landmark conocido `i` (log-pesos, normalización estable, diferencia angular con
  wraparound). Sólo corrige si hubo movimiento (`update_min_d/ a`).
- **Resampling** low-variance cuando `n_eff < N/2`, con **roughening** (jitter `rough_xy=0.03`,
  `rough_yaw=0.02`) post-resampling para evitar el empobrecimiento de la nube.
- `num_particles=350`. Inicialización por `/initialpose` (RViz "2D Pose Estimate").
- Salidas: `/mcl_pose`, `/particlecloud`, **TF `map→odom`** (a 30 Hz).
- **`transform_tolerance=0.1 s` (truco de AMCL):** la TF `map→odom` se publica con el stamp
  adelantado 0.1 s, para que siga siendo válida para lookups a tiempos un poco futuros (scan/
  observaciones llegan con stamp más nuevo que la última TF). Evita el parpadeo de "extrapolation
  into the future" en RViz (ver `02_guia_ejecucion.md` §5).

### 3.5 `global_planner` (A* — consigna 1.5)
- A* **8-conexo** sobre la grilla. **Inflado** de obstáculos por `robot_radius=0.18 m`
  (distancia de Chebyshev por BFS multi-fuente, sin dependencias externas).
- **Ruta segura:** penalización de cercanía (`clearance_weight`, `clearance_max`) que encarece
  pasar pegado a las paredes → el robot prefiere el centro de los pasillos.
- `allow_unknown=False`: las celdas desconocidas (gris) se tratan como obstáculo (seguridad).
- **Atajo line-of-sight** (Bresenham) que simplifica la ruta a pocos waypoints → seguimiento más
  suave. El último waypoint se fuerza exactamente al `goal_pose` (posición + ángulo).
- Es un nodo **puro/sin estado**: planea cuando recibe `/plan_request` y publica `/plan`.

### 3.6 `obstacle_monitor` (obstáculos NO mapeados — consigna 1.9)
- La clave: distinguir paredes conocidas de obstáculos nuevos. Por cada eco del LIDAR dentro de un
  cono frontal (`cone_halfangle=0.6 rad`) y cerca (`danger_dist=0.45 m`), proyecta el punto al
  frame `map` (con la TF del MCL) y consulta la celda: si el mapa la marca **libre** y el LIDAR ve
  algo ahí → **obstáculo nuevo**. Si la celda es pared conocida → se ignora.
- Si hay ≥ `min_points=3` puntos nuevos → publica `/obstacle_detected=True`.

### 3.7 `state_machine` (FSM + control — consigna 1.6, 1.7, 1.8, 1.11)
Máquina de estados (idéntica al diagrama de `04_...md`):

```
IDLE --initialpose--> LOCALIZING --pose estable--> WAITING_GOAL
WAITING_GOAL --goal_pose--> PLANNING --plan OK--> FOLLOWING
FOLLOWING --llegó a la posición--> ALIGNING_ANGLE --orientación OK--> GOAL_REACHED --> WAITING_GOAL
FOLLOWING --obstáculo--> AVOIDING --despejado--> PLANNING (re-planea)
FOLLOWING/ALIGNING/GOAL_REACHED --goal_pose nuevo--> PLANNING (re-planea, 1.8)
```

- **Seguimiento (1.6):** *pure-pursuit* con carrot a `lookahead=0.30 m`; avanza más lento cuanto
  mayor es el error angular (gira primero, suave). `v_max=0.18`, `w_max=1.2`.
- **Ángulo final (1.7):** estado `ALIGNING_ANGLE` que gira en el lugar hasta el yaw del goal
  (`goal_yaw_tol=0.08 rad`); posición con `goal_xy_tol=0.12 m`.
- **Re-planeo (1.8):** un `goal_pose` nuevo en cualquier estado activo → vuelve a `PLANNING`.
- **Evasión (1.9):** `AVOIDING` frena/retrocede/gira un tiempo y dispara un re-planeo.
- Publica `/nav_state` (`String`) para ver el estado en RViz/consola.

---

## 4. Cómo se llegó a que el MCL funcione (bitácora de tuning)

El MCL **divergía** al navegar. Tres correcciones, cada una verificada con datos:

1. **Desacople del sensor** (§3.3). Causa raíz: el sensor proyectaba landmarks con la *estimación*
   del MCL → realimentación → al derivar un poco, los landmarks "dejaban de verse" y el filtro
   quedaba en dead-reckoning → divergencia sin retorno (estimación a >2 m). Fix: proyectar con la
   pose verdadera (`odom`).
2. **Densidad 14 → 36 landmarks** (§3.2). Con 14 el posterior era **multimodal** (simetrías de la
   casa) y la media pesada saltaba metros entre modos. Con 36 el problema desaparece.
3. **Roughening + 350 partículas** (§3.4). Al converger muy ajustado la nube colapsaba y perdía el
   lock ante una discrepancia transitoria. El jitter post-resampling mantiene diversidad → ahora
   **se recupera** de picos en vez de diverger.

**Resultado:** tracking de **0.02–0.18 m** en rutas normales (antes 0.65 m medio / 2.9 m pico).
Ver números completos en [`03_resultados_pruebas.md`](03_resultados_pruebas.md).

---

## 5. Archivos del paquete

```
tp_b_navigation/
├── tp_b_navigation/
│   ├── map_loader.py          landmark_publisher.py   landmark_sensor.py
│   ├── mcl_localization.py     global_planner.py       obstacle_monitor.py
│   ├── state_machine.py        utils.py
├── config/
│   ├── landmarks.yaml          (36 landmarks)
│   └── parte_b.rviz            (config de RViz de Parte B)
├── launch/
│   ├── parte_b.launch.py             (pila completa: nav + RViz)
│   └── parte_b_localization.launch.py (sólo localización)
├── setup.py  package.xml  resource/
```
