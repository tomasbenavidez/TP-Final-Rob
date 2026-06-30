# Parte B — Documentación (navegación autónoma, Sistema 3)

Bundle autocontenido de la Parte B: cómo está hecha, cómo correrla y qué resultados dio.

## Documentos
1. **[01_implementacion.md](01_implementacion.md)** — decisiones de diseño: los 7 nodos, la cadena
   de TF, la máquina de estados, y la bitácora de cómo se logró que el MCL funcione.
2. **[02_guia_ejecucion.md](02_guia_ejecucion.md)** — **cómo correr todo** (Gazebo + pila + RViz),
   qué se ve en cada paso, comandos, troubleshooting de este Mac, y la **lista de screenshots que
   faltan** (para sacar con la compu).
3. **[03_resultados_pruebas.md](03_resultados_pruebas.md)** — resultados medidos en `custom_casa` y
   `custom_casa_obs`, el antes/después del tuning del MCL, y el resumen contra la consigna.

## Para correr rápido (resumen)
```bash
# Terminal 1 (Gazebo)
source docs/parte_b/scripts/setup_parte_b.sh
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py      # o custom_casa_obs.launch.py
# Terminal 2 (pila de Parte B + RViz)
source docs/parte_b/scripts/setup_parte_b.sh
cd tp_final_ws && colcon build --packages-select tp_b_navigation
ros2 launch tp_b_navigation parte_b.launch.py
# En RViz: "2D Pose Estimate" y después "2D Goal Pose"
```
> En este Mac es **imprescindible** sourcear `setup_parte_b.sh` (arma el entorno y fija CycloneDDS
> en loopback). Detalles en `02_guia_ejecucion.md`.

## Carpetas
- `scripts/` — `setup_parte_b.sh` (entorno), `cyclonedds_loopback.xml` (DDS), `gen_landmarks.py`
  (generador configurable; actualmente produce 60 landmarks).
- `img/` — gráficos de los resultados (mapa + landmarks + ruta + trayectoria real vs MCL).

## Estado al cierre
Ciclo completo funcionando en `custom_casa` (localización MCL + A* + seguimiento + ángulo final +
re-planeo). Detección y evasión de obstáculos no mapeados funcionan en `custom_casa_obs` (sin
colisión), con una limitación conocida de eficiencia (falta un *costmap local*; ver
`03_resultados_pruebas.md` §3.3). Pendiente: los screenshots de RViz/Gazebo (la compu con GUI).
