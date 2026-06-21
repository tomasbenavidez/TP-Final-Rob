# Parte B — Resultados de las pruebas

Pruebas corridas contra la simulación de Gazebo en este Mac. Cada número viene de
correr la pila real (no es teórico). Las imágenes están en [`img/`](img/) y se generaron con
matplotlib (mapa + landmarks + ruta + trayectoria real vs MCL) porque RViz no renderiza ventana en
el entorno de desarrollo (ver [`02_guia_ejecucion.md`](02_guia_ejecucion.md) §6).

---

## 1. `custom_casa` — ciclo completo (sin obstáculos)

### 1.1 Pila de navegación (con localización perfecta, para validar planner+follower+FSM)
Reemplazando el MCL por una TF `map→odom` identidad (localización = ground-truth), para aislar la
navegación:

- Goal `(-2.0, 2.2, 180°)` desde `(0,0)`. La FSM recorrió
  `IDLE→LOCALIZING→WAITING_GOAL→PLANNING→FOLLOWING (22 s)→ALIGNING_ANGLE→GOAL_REACHED`.
- **Llegó a `(-1.90, 2.20, -177°)`** → error **~10 cm / ~3°** (dentro de `goal_xy_tol=0.12`,
  `goal_yaw_tol=0.08`).
- A* rodeó correctamente la sala central (ver `img/02_navegacion_localizacion_perfecta.png`).

→ **Planner + path-following + ángulo final + FSM: OK.**

### 1.2 Ciclo completo con MCL real
- Robot cruzó toda la casa, ~5 m, desde `(2.35, 0.11)` a `(-1.95, -2.28)`.
- **Llegó al goal con error ~6 cm**; el MCL estimaba a **~1.5 cm** de la pose real en el goal.
- La FSM re-planeó ante detecciones espurias y completó posición + ángulo final.

→ **Ciclo completo (localización + planificación + seguimiento + ángulo) con MCL real: OK.**

---

## 2. Tuning de localización (MCL) — antes vs después

Medición: manejar el robot y comparar la pose estimada (TF `map→base`) contra la pose real
(`/odom`, que en Gazebo es ~ground-truth).

| Configuración | Error medio | Error máx | Comportamiento |
|---|---|---|---|
| **Antes** (14 landmarks, sensor acoplado a la estimación, sin roughening) | 0.65 m | 2.9 m | **diverge** y no se recupera (ver `img/03_mcl_antes_tuning_14lm.png`: la estimación naranja salta metros) |
| **Después** (36 landmarks, sensor desacoplado, roughening, 350 part.) | **0.02–0.18 m** en rutas normales | recupera de picos | trackea pegado (`img/01_...`, `img/02_...`) |

Las tres correcciones (desacople del sensor / densidad 14→36 / roughening) están explicadas en
[`01_implementacion.md`](01_implementacion.md) §4. Caso patológico: girar en el lugar en círculos
cerrados sostenidos puede dar un pico de ~1 m, pero **se recupera** (antes divergía).

---

## 3. `custom_casa_obs` — obstáculos NO mapeados (consigna 1.9)

Mundo `casa_o.world`: agrega **valijas** que no están en el mapa, en
`(-0.36, 0.79)`, `(2.17, -0.26)`, `(-0.04, -2.18)` y un **cluster** en `(-2.1/-1.9/-1.6, 1.9)`.

### 3.1 Goal que NO cruza obstáculos
Goal `(0.3, -2.6)` desde `(0,0)`: el robot navegó y **llegó sin gatillar evasión** (la ruta no
pasaba cerca de ninguna valija). `obstacle_detected=false` todo el tiempo. → la detección no da
falsos positivos con las paredes mapeadas.

### 3.2 Goal que SÍ cruza obstáculos
Goal `(-2.0, 2.2)` (la ruta pasa por la valija de `(-0.36, 0.79)` y termina detrás del cluster):

- **El `obstacle_monitor` detectó correctamente** la valija no mapeada (`obstacle_detected=true`).
- La FSM entró en **`AVOIDING`** (8 maniobras a lo largo del encuentro): frena, retrocede, gira y
  re-planea.
- **El robot esquivó sin chocar** y siguió (`obstacle_detected` volvió a `false`).
- **El MCL se mantuvo preciso** durante las maniobras: error medio **0.16 m**, máx 0.90 m
  (799 muestras) — las rotaciones de evasión no lo hicieron diverger.
- Ver `img/04_evasion_obstaculos.png`: trayectoria roja esquivando, MCL (naranja) pegado.

→ **Detección de obstáculos no mapeados + evasión reactiva sin colisión + MCL robusto: OK.**

### 3.3 Limitación encontrada (importante)
Cuando un obstáculo se sienta **justo sobre la única/mejor ruta** (caso del cluster de valijas
tapando la entrada al cuarto del goal), la navegación se vuelve **lenta y sinuosa**: el
`global_planner` planifica sobre el **mapa estático** (no conoce las valijas), así que re-planifica
la **misma** ruta bloqueada → el robot avanza sólo por los empujones reactivos de `AVOIDING` →
oscila/serpentea y tarda mucho (en una corrida no llegó al goal detrás del cluster en >2 min).

**No choca y no se rompe**, pero es ineficiente. **Mejora futura clara:** agregar los obstáculos
detectados a un *costmap local* que el planner sume al mapa estático, para que A* rodee el obstáculo
en vez de re-planear contra él. Es el siguiente incremento natural.

---

## 4. Resumen contra la consigna

| Requisito | Estado |
|---|---|
| 1.1 Sistema 3 (landmarks de cámara virtuales) | ✅ 36 landmarks + sensor con FOV y oclusión |
| 1.2 Localización inicial (`initialpose`) | ✅ "2D Pose Estimate" → MCL siembra |
| 1.3 Objetivo (`goal_pose`) | ✅ "2D Goal Pose" |
| 1.4 Localización continua (filtro probabilístico) | ✅ MCL, 0.02–0.18 m en rutas normales |
| 1.5 Planificación (A* / inflado / ruta segura) | ✅ A* + inflado + clearance + atajo LOS |
| 1.6 Seguimiento suave | ✅ pure-pursuit |
| 1.7 Ángulo final | ✅ estado `ALIGNING_ANGLE` |
| 1.8 Re-planeo (goal nuevo) | ✅ |
| 1.9 Obstáculos no mapeados | ⚠️ detecta + esquiva sin chocar; ineficiente si el obstáculo tapa la ruta (falta costmap local) |
| 1.10 `custom_casa` / `custom_casa_obs` | ✅ probados (`custom_casa_obs2` no está instalado) |
| 1.11 Máquina de estados | ✅ FSM completa con todas las transiciones |
| 1.12 Criterios (llegar, seguridad, incertidumbre) | ✅ en `custom_casa`; ⚠️ eficiencia con obstáculos densos |

---

## 5. Pendientes
- Screenshots de RViz/Gazebo (requiere la compu con GUI — ver `02_guia_ejecucion.md` §6).
- *Costmap local* para que el planner rodee obstáculos no mapeados (mejora §3.3).
- Bajar el pico transitorio del MCL en traverses largos.
- Integrar al informe y el video (entregables 2 y 3).
