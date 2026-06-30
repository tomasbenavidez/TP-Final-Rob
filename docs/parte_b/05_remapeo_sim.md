# Parte B — Re-mapeo del entorno simulado (usar el mapa de Parte A)

Cómo Parte B pasa de usar el **mapa provisto por el profesor** (`mapas/map.yaml`) a usar
un mapa **generado con la lógica de mapeo de Parte A**, pero del **entorno simulado** de
Gazebo (`custom_casa`), como pide la consigna del **Sistema 3**:

> *"Con esos landmarks ficticios, **repetir el proceso de mapeo** para tener un mapa del
> **entorno simulado** coherente."* (`01_parte_b_consigna.md`, §1.1)

Parte A construye su mapa a partir del **rosbag del TurtleBot4** (mundo real, cámara+ArUco+LIDAR,
`occupancy_grid_node`). Ese mapa **no coincide** con el mundo Gazebo donde navega Parte B. Por eso
se **repite el mapeo en simulación**: mismo algoritmo, datos de Gazebo.

## El nodo `sim_mapper` (reusa el mapeo de Parte A)

`tp_b_navigation/sim_mapper.py` **porta el núcleo de `occupancy_grid_node` de Parte A**
(`tp_a_slam_aruco`): modelo de sensor inverso + **log-odds** (mismas constantes `L_OCC=0.85`,
`L_FREE=-0.40`, sat. `±5`) + **Bresenham** + export `.pgm/.yaml` idéntico (umbrales `occ=0.60`,
`free=0.40`, `negate:0`). Está portado (con atribución) igual que `landmark_sensor` está portado
de tp5, para que Parte B quede **autocontenida**.

Única diferencia con Parte A: la pose de cada barrido **no** viene de un JSON de trayectoria del
Graph SLAM (en Gazebo no hay ArUco), sino de la **TF en vivo** `mapping_frame → frame del LIDAR`.
En Gazebo la odometría es ~ground-truth, así que `odom ≡ map` en el origen y el mapa queda
alineado con lo que asume Parte B. Validado: las paredes del mapa generado **se superponen** con
las del mapa del profe (mismo frame, misma geometría).

## Cómo generar el mapa (teleop — flujo estándar de SLAM)

El robot se maneja **a mano** para cubrir toda la casa (es como se hacen los mapas SLAM en la
práctica; la cobertura autónoma se traba en las puertas angostas de `custom_casa`). Tres
terminales, **todas** con el entorno sourceado (`source docs/parte_b/scripts/setup_parte_b.sh`):

```bash
# T1 — simulación (robot spawnea en el origen, libre)
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py

# T2 — mapeo (sim_mapper; publica /map para verlo en RViz; exporta al cerrar)
ros2 launch tp_b_navigation sim_mapping.launch.py
#   por defecto exporta /tmp/map_sim.{pgm,yaml}

# T3 — teleop: manejar el TB3 por TODA la casa
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Recorrido sugerido para cobertura completa** (la casa tiene un muro en `y≈-1.5` que separa
norte/sur; la **puerta al sur está en el corredor ESTE, x≈+2.4**):

1. Girar en el lugar en el spawn (barre el cuarto central).
2. Recorrer el perímetro norte (paredes de arriba, izquierda, derecha) y entrar al cuarto oeste.
3. Volver al centro → ir al **este** (x≈2.4) → **bajar por el corredor este** cruzando `y=-1.5`.
4. Barrer el cuarto sur (esquinas y paredes de abajo).
5. Volver al norte. Pasar **cerca de las paredes** (el LIDAR llega a 3.5 m; conviene no quedar
   lejos de cada pared).

> Tip: dejá abierto RViz con un display **Map** sobre `/map` (fixed frame `map`) para ver qué
> falta y guiar el recorrido. Cuando ya no queden zonas grises dentro de la casa, listo.

**Exportar:** `Ctrl+C` en **T2** → `sim_mapper` escribe `/tmp/map_sim.pgm` +
`/tmp/map_sim.yaml` por defecto.

## Re-mapear más tarde (y si hay que reconectar algo)

**Para volver a mapear** (mismas 3 terminales que arriba):

```bash
# T1
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
# T2, desde la raíz del repositorio, para actualizar el mapa versionado
ros2 launch tp_b_navigation sim_mapping.launch.py \
  map_output:="$(pwd)/mapas/map_sim"
# T3
ros2 run teleop_twist_keyboard teleop_twist_keyboard
#   manejar la casa  ->  Ctrl+C en T2  ->  SOBRESCRIBE mapas/map_sim.{pgm,yaml}
```

`map_loader.py` y `parte_b.launch.py` usan por defecto
`share/tp_b_navigation/maps/map_sim.yaml`, un recurso instalado por `colcon`. Si se regenera
`mapas/map_sim`, hay que repetir `colcon build --packages-select tp_b_navigation` para refrescar
la copia instalada. Como alternativa, se puede probar sin rebuild desde la raíz del repositorio:

```bash
ros2 launch tp_b_navigation parte_b.launch.py \
  map_yaml:="$(pwd)/mapas/map_sim.yaml"
```

**Landmarks:** si re-mapeás **el mismo mundo** (`custom_casa`, robot spawneando en el origen), la
geometría no cambia → los landmarks siguen cayendo sobre las paredes → **NO hace falta
regenerarlos**. Sólo regenerar si cambiás de mundo o querés otra densidad:

```bash
python3 docs/parte_b/scripts/gen_landmarks.py                          # lee mapas/map_sim.yaml
cd tp_final_ws && colcon build --packages-select tp_b_navigation       # landmarks SÍ necesita rebuild
```

> Tanto el mapa predeterminado como los landmarks son recursos del paquete instalado. Los cambios
> en `mapas/` o `config/` requieren rebuild para actualizar `install/`; `map_yaml` permite probar
> un mapa fuente directamente.

> Recordá el **gotcha de DDS** (ver más abajo): si tras varias corridas la FSM queda en LOCALIZING,
> matá todo + `ros2 daemon stop && ros2 daemon start` + relanzá. No es el mapa.

## Cómo Parte B usa el mapa generado

`map_loader` y `parte_b.launch.py` aceptan el argumento `map_yaml`. Para navegar con el mapa
nuevo:

```bash
ros2 launch tp_b_navigation parte_b.launch.py \
    map_yaml:="$(pwd)/mapas/map_sim.yaml"
```

(Una vez validado el mapa nuevo, se puede dejar `map_sim.yaml` como default del launch y
regenerar `config/landmarks.yaml` sobre su geometría — ver `gen_landmarks.py`.)

## Estado (COMPLETO ✅)

- `sim_mapper` + `sim_mapping.launch.py`: hechos y probados.
- **Mapa generado por teleop:** `mapas/map_sim.{pgm,yaml}` (220×220 @ 0.05, origin [-5,-5]).
  **100% de cobertura** de las paredes del mapa del profe (norte y sur), alineación casi perfecta
  (1822 celdas de pared vs 1660 del profe). Único defecto cosmético: un pequeño remolino de ruido
  sobre la pared oeste (donde el robot quedó girando), inofensivo para la navegación.
- `config/landmarks.yaml`: **regenerado** (60 landmarks por farthest-point sampling sobre
  `map_sim`; `gen_landmarks.py` ahora apunta a `map_sim.yaml` por default).
- `map_loader.py` y `parte_b.launch.py`: **default instalado =
  `share/tp_b_navigation/maps/map_sim.yaml`** (el del profe queda disponible vía
  `map_yaml:=.../map.yaml`).
- **Validación end-to-end (✅):** con la pila levantada sobre `map_sim`, el ciclo completo corre:
  IDLE→LOCALIZING→WAITING_GOAL→PLANNING→FOLLOWING→ALIGNING_ANGLE→GOAL_REACHED. El robot fue de
  (0,0) al goal (1.2,0) y llegó dentro de ~9 cm. MCL localiza y el planner planea sobre el mapa
  nuevo.

> **Gotcha de esta Mac:** tras una sesión larga con muchos procesos, el DDS se degrada (nodos
> "suscriptos pero 0 datos"; la FSM queda atascada en LOCALIZING porque no llega `/scan` →
> MCL no publica `map→odom`). Solución: matar todo, `ros2 daemon stop && ros2 daemon start`,
> y relanzar. No es problema del mapa.

## Herramientas de desarrollo (no son nodos de la pila)

En `docs/parte_b/scripts/`: `coverage_drive.py` (wanderer reactivo) y `tour_drive.py` /
`goal_tour.py` (recorridos dirigidos). Sirven para cobertura automática parcial, pero **el método
recomendado y confiable es el teleop**.
