# Parte C — exploración informativa y conos rojos

## Funcionalidad

Parte C agrega una capa de misión sin reemplazar la navegación estable de Parte B:

1. `mission_manager` mantiene qué parte del mapa ya fue observada.
2. Genera poses alcanzables con distintas orientaciones y estima por ray-casting qué vería la cámara.
3. Selecciona la acción con mayor utilidad:

   `U = 0.55 cobertura + 0.20 localización - 0.20 camino - 0.05 riesgo`.

4. `red_cone_detector` segmenta las dos bandas rojas de HSV, limpia la máscara, valida la geometría y exige confirmación temporal.
5. La distancia se obtiene de depth alineado. Si no hay suficientes píxeles válidos, se usa `fy * cone_height_m / alto_px`.
6. Una detección confirmada interrumpe el objetivo exploratorio y genera poses de aproximación alrededor del cono. Sólo se acepta una pose libre, visible y alcanzable mediante A*.
7. La misión termina en `FOUND` al alcanzar el primer cono.

La detección nunca se convierte en un comando de velocidad directo. `state_machine` sigue siendo el único productor de `/cmd_vel`.

## Estados e interfaces

`/mission/status` publica `IDLE`, `EXPLORING`, `APPROACHING`, `FOUND` o `NOT_FOUND`.
La misión se controla con los servicios `/mission/start` y `/mission/cancel`.

Internamente:

- `/mission_goal`: objetivo elegido por exploración o aproximación.
- `/mission_cancel`: cancelación segura.
- `/mission/coverage_markers`: celdas libres ya observadas por la cámara, para RViz.
- `/mission/frontier_markers`: candidatos frontier y objetivo elegido, para RViz.
- `/navigation_result`: `REACHED`, `PLAN_FAILED`, `TIMEOUT` o `PREEMPTED`.
- `/red_cone_pose`: pose confirmada en `map`, con covarianza según depth/monocular.
- `/red_cone/debug_image` y `/red_cone/mask`: diagnóstico para calibración.

## Perfiles

### Simulación

```bash
ros2 launch tp_c_mission parte_c_sim.launch.py world:=casa
# o world:=casa_obs
```

El launch usa el mapa y MCL de Parte B, un Burger con cámara RGB-D y tres conos: rojo, amarillo y azul. Los launch originales de Parte B conservan el Burger estándar por defecto.
RViz abre con `parte_c_sim.rviz`, que muestra `/plan`, `/mission_goal`, cobertura explorada,
frontiers candidatos y la pose confirmada del cono.

### Rosbag de visión

```bash
ros2 launch tp_c_mission parte_c_bag.launch.py \
  rgb_topic:=/rgb/image_raw \
  depth_topic:=/depth/image_raw \
  camera_info_topic:=/rgb/camera_info
```

Los umbrales iniciales están en `config/parte_c.yaml`; deben calibrarse contra el bag sin convertirlos en constantes del código.
El perfil de bag abre por defecto RViz en `odom`, con TF, LIDAR (`/tb4_0/scan`),
odometría con traza (`/tb4_0/odom`), pose confirmada del cono, imagen debug y máscara.
El mapa (`/map`) y el plan (`/plan`) quedan cargados pero desactivados, porque un rosbag
de visión puro puede no traer navegación activa.

Para correr sólo los nodos de percepción, sin GUI:

```bash
ros2 launch tp_c_mission parte_c_bag.launch.py launch_rviz:=false
```

### TurtleBot4 real

Primero ejecutar Parte A y conservar:

- `mapa.yaml` + `mapa.pgm`;
- `trayectoria.json`, que contiene IDs y posiciones optimizadas de ArUco.

Luego:

```bash
ros2 launch tp_c_mission parte_c_real.launch.py \
  map_yaml:=/tmp/mapa.yaml \
  landmark_map_file:=/tmp/trayectoria.json \
  depth_topic:=/oakd/depth/aligned/image_raw
```

El perfil real no inicia `landmark_sensor`. `aruco_mcl_adapter` transforma detecciones reales a range/bearing con ID, y MCL realiza la asociación contra el JSON de Parte A.

## Seguridad y fallos

- Sin mapa o pose MCL, `/mission/start` falla sin mover el robot.
- Si A* no puede alcanzar un candidato, se descarta y se evalúa otro.
- Si el cono se ve por una abertura, el objetivo final se valida sobre la grilla y el camino rodea paredes conocidas.
- Ante timeout o cobertura agotada, se publica `NOT_FOUND` y se cancela navegación.
- `custom_casa_obs` mantiene la evasión reactiva de Parte B y agrega `/dynamic_obstacles` compacto para que el siguiente A* rodee obstáculos no mapeados ya detectados sin bloquear pasillos enteros.
- `custom_casa_obs2` no forma parte de los entornos soportados.

## Verificación

```bash
python3 tp_final_ws/src/tp_b_navigation/test/test_planner_core.py -v
python3 tp_final_ws/src/tp_b_navigation/test/test_landmark_io.py -v
python3 tp_final_ws/src/tp_c_mission/test/test_cone_perception.py -v
python3 tp_final_ws/src/tp_c_mission/test/test_information_exploration.py -v
python3 tp_final_ws/src/tp_c_mission/test/test_parte_c_contracts.py -v
python3 -m compileall -q tp_final_ws/src/tp_b_navigation tp_final_ws/src/tp_c_mission
git diff --check
```

Con ROS 2, ejecutar además el build completo y probar ambos mundos. El bag de visión y el hardware real no están versionados, por lo que su validación requiere esos recursos externos.
