# Parte A - Handoff

Este documento resume el estado de la rama para que otra persona pueda retomar
el trabajo sin depender de contexto oral ni de rutas locales.

## Enfoque

La Parte A apunta a construir un mapa del entorno usando dos pasadas sobre un
rosbag:

1. **SLAM visual con ArUco:** el detector estima observaciones de landmarks y
   `graph_slam_node` las usa junto con odometria para construir una trayectoria
   optimizada.
2. **Mapeo LIDAR:** `occupancy_grid_node` carga la trayectoria optimizada y
   proyecta los scans del LIDAR para escribir un mapa de ocupacion.

La idea de esta rama es acercarse a la opcion con camara + LIDAR + landmarks del
enunciado. No es todavia una implementacion cerrada.

## Estado actual

- Hay un paquete nuevo, `tp_slam_interfaces`, con mensajes internos para pasar
  observaciones visuales y diagnosticos.
- `aruco_detector_node` publica observaciones aceptadas en
  `/landmark_observations` y diagnosticos en `/landmark_detection_stats`.
- `graph_slam_node` consume esas observaciones, crea candidatos de landmarks y
  publica trayectoria, poses y landmarks optimizados.
- `parte_a_slam.launch.py` arma la primera pasada e incluye RViz dentro del
  pipeline.
- `parte_a_mapa.launch.py` arma la segunda pasada usando el JSON de trayectoria.

La ultima prueba manual reportada dejo bugs pendientes: los landmarks no se
estaban cargando/confirmando de forma confiable.

## Organizacion del codigo

- `tp_final_ws/src/tp_slam_interfaces/msg/`: contratos ROS para observaciones de
  landmarks y diagnosticos visuales.
- `tp_final_ws/src/tp_slam_aruco/launch/`: launch files de primera y segunda
  pasada.
- `tp_final_ws/src/tp_slam_aruco/tp_slam_aruco/aruco_detector_node.py`: nodo de
  deteccion ArUco, calibracion, TF camara -> base y publicacion de observaciones.
- `tp_final_ws/src/tp_slam_aruco/tp_slam_aruco/graph_slam_node.py`: construccion
  del grafo, keyframes, candidatos de landmarks, optimizacion y export de
  trayectoria.
- `tp_final_ws/src/tp_slam_aruco/tp_slam_aruco/occupancy_grid_node.py`: segunda
  pasada de mapeo LIDAR.
- Modulos auxiliares como `slam_landmarks.py`, `slam_timing.py`,
  `slam_graph_diagnostics.py`, `slam_motion_filter.py` y `slam_publish.py`
  concentran logica que conviene mantener testeable fuera de los nodos ROS.

## Bugs conocidos y puntos a revisar

El foco inicial deberia ser el flujo de landmarks:

- Verificar que `max_reprojection_error_px` del detector y
  `max_candidate_reprojection_error_px` del SLAM esten coordinados desde el
  launch. Si el detector filtra demasiado antes de publicar, el SLAM nunca ve
  candidatos suficientes.
- Confirmar con `ros2 topic echo /landmark_observations` que realmente llegan
  observaciones durante el bag.
- Revisar en `LandmarkCandidateManager` los filtros de edad, parallax, cantidad
  minima de observaciones y reset por observacion mala.
- Verificar que `camera_info` llegue antes de esperar detecciones validas. Si no
  llega, el detector descarta frames salvo que se habilite el fallback YAML.
- Verificar TF camara -> `base_link`. Si no esta disponible y el fallback esta
  deshabilitado, las detecciones se rechazan por TF.
- Mirar `/landmark_detection_stats` para separar problemas de deteccion visual
  de problemas del grafo.

## Proximos pasos recomendados

1. Construir ambos paquetes:
   `colcon build --packages-select tp_slam_interfaces tp_slam_aruco`.
2. Correr los tests disponibles con `colcon test` o `pytest` dentro de un entorno
   ROS preparado.
3. Reproducir solo la primera pasada y registrar:
   `/landmark_detection_stats`, `/landmark_observations`, logs de
   `aruco_detector_node` y logs de `graph_slam_node`.
4. Si `/landmark_observations` esta vacio, depurar detector, calibracion, TF y
   filtros de area/proyeccion.
5. Si `/landmark_observations` tiene datos pero no aparecen landmarks
   optimizados, depurar `LandmarkCandidateManager` y los filtros de edad/parallax.
6. Recien despues de estabilizar landmarks, volver a mirar calidad de
   trayectoria y mapa LIDAR.

## Verificacion pendiente

En el entorno donde se preparo este handoff no estaban disponibles `colcon` ni
`pytest`, por lo que no se pudo correr la suite completa localmente. La
verificacion minima posible fue compilar sintaxis Python con:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile tp_final_ws/src/tp_slam_aruco/tp_slam_aruco/*.py
```
