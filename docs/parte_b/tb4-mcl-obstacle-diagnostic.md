# Diagnóstico TB4 de MCL y obstáculos

Fecha: 2026-06-30.

## Objetivo y reproducción

El smoke de Parte B reproduce `tp_final_ws/bags/laberinto` en un dominio ROS
aislado, con logs propios y la salida de velocidad remapeada para impedir
publicación al `/cmd_vel` real. Para repetirlo:

```bash
export TP_TB4_TEST_BAG="$PWD/tp_final_ws/bags/laberinto"
cd tp_final_ws
source install/setup.bash
python -m pytest \
  src/tp_b_navigation/test/test_tb4_runtime_smoke.py \
  -k parte_b -vv
```

Para una captura diagnóstica manual, grabar juntos:

```text
/odom
/mcl_pose
/particlecloud
/observed_landmark_ids
/obstacle_detected
/obstacle_monitor_healthy
/dynamic_obstacles
/nav_state
/cmd_vel
/tf
/tf_static
```

Usar los tópicos namespaced resueltos por el perfil cuando corresponda. No
grabar ni publicar sobre el `/cmd_vel` físico durante un smoke.

## Hallazgos en orden causal

1. La primera falla fue de localización: el bag publicaba odometría
   best-effort y MCL la solicitaba reliable. No llegaban predicciones
   odométricas ni aparecía `map -> odom`.
2. Tras hacer compatible la suscripción de odometría, MCL avanzó y publicó
   estimación y nube entre observaciones ArUco.
3. El siguiente síntoma fue `/obstacle_monitor_healthy: false`: el scan más
   reciente podía llegar antes que el TF de su timestamp. El monitor ahora
   conserva ese scan acotadamente y lo reintenta cuando llega TF, sin consultar
   indiscriminadamente el TF “latest”.

El orden observado fue, por tanto, divergencia/ausencia de localización antes
que inserción falsa de obstáculos. No hubo evidencia que justificara cambiar
ruido MCL, A*, inflación, TTL ni parámetros de evasión, y no se modificaron.

## Resultado local

- Smoke conjunto A/B/C: 3 pruebas aprobadas en 116.95 s.
- Suite ROS: 215 aprobadas y 4 omitidas.
- Parte B no produjo movimiento físico ni publicó al `/cmd_vel` real.
- El bag valida transporte, predicción MCL y salud del monitor; no sustituye
  los gates físicos B1–B3.

## Gates físicos pendientes

| Gate | Procedimiento mínimo | Estado |
|---|---|---|
| B1 | Localización con movimiento autónomo deshabilitado | Pendiente |
| B2a | Un goal corto, libre y a velocidad reducida | Pendiente |
| B2b | Múltiples goals tras aprobar B2a | Pendiente |
| B3a | Detectar obstáculo con robot detenido | Pendiente |
| B3b | Replanificar sin movimiento | Pendiente |
| B3c | Evasión reducida y recuperación de MCL | Pendiente |

Antes de cada gate: cancelar, publicar velocidad cero, comprobar que el robot
está detenido y mantener accesible la parada de emergencia.
