# Correcciones de odometría y landmarks virtuales

**Objetivo:** Corregir Parte B y Parte C simulada para usar odometría calculada,
una cámara virtual frontal con oclusión robusta y 60 landmarks, sin alterar los
contratos reales de las Partes A y C.

## Cambios

- Exponer `odom_topic` por launch: `/calc_odom` para TB3 simulado y
  `/tb4_0/odom` para TB4 real.
- Separar en MCL la odometría de movimiento de la referencia usada para publicar
  `map -> odom`; reservar `/odom` de Gazebo como verdad simulada.
- Activar el publicador existente de `/calc_odom` en `custom_casa` y
  `custom_casa_obs`, sin agregar ruido artificial.
- Modelar una cámara virtual frontal con TF preferida, extrínseca fallback,
  `camera_fov=1.05`, rango configurable y observaciones expresadas en
  `base_footprint`.
- Reemplazar la oclusión permisiva por una comparación conservadora con
  `occlusion_tol=0.08`.
- Parametrizar el generador y regenerar 60 landmarks virtuales únicos sobre
  superficies de pared.
- Actualizar pruebas y documentación manteniendo `/tb4_0/odom` y los ArUco reales
  de Parte A y Parte C real.

## Verificación

- Pruebas unitarias para FOV, extrínseca, rango y oclusión.
- Pruebas de contratos de launch y exactamente 60 landmarks.
- Verificaciones portables, `compileall`, `git diff --check`, build ROS y pytest
  completo cuando ROS 2 esté disponible.
