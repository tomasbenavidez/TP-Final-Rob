# Diseño: portabilidad e integración documental de Parte B

## Objetivo

Hacer que la rama `feat/parte-B` pueda clonarse y ejecutarse desde cualquier directorio sin depender de rutas personales, manteniendo intactos los algoritmos y contratos ROS que ya funcionan.

## Alcance

- Incluir `map.yaml` y `map.pgm` en los recursos instalados de `tp_b_navigation` durante `colcon build`.
- Resolver el mapa predeterminado mediante `get_package_share_directory('tp_b_navigation')`.
- Derivar rutas de scripts auxiliares desde la ubicación del propio script.
- Reemplazar rutas absolutas en documentación por rutas relativas al repositorio o al workspace.
- Renombrar `AGENT.md` a `AGENTS.md` y actualizarlo para describir Partes A y B.
- Reescribir el README principal como guía portable del flujo Parte A → mapa → Parte B.
- Agregar verificaciones automáticas que detecten rutas personales y contratos de empaquetado rotos.

## Diseño del mapa instalado

Los archivos fuente continúan en `mapas/`. `setup.py` los copia a `share/tp_b_navigation/maps` dentro del install space. Tanto los launch como `map_loader` buscan el mapa instalado mediante el índice de paquetes de ROS 2. `map.yaml` conserva su referencia relativa a `map.pgm`, por lo que ambos deben instalarse juntos.

## Scripts y documentación

`setup_parte_b.sh` calcula la raíz del repositorio desde `docs/parte_b/scripts/` y permite sobrescribir las rutas externas del entorno mediante variables. `gen_landmarks.py` usa `pathlib.Path(__file__)` para encontrar el mapa y la configuración. Los ejemplos de README y guías parten de la raíz del clon o de `tp_final_ws`, sin asumir usuario ni sistema operativo.

## Límites de seguridad

No se cambian tópicos, tipos de mensajes, frames, parámetros numéricos, algoritmos de SLAM/localización/planificación/control ni el orden de ejecución. Parte A y Parte B permanecen como etapas separadas conectadas por el mapa persistido.

## Verificación

- Un test escanea archivos ejecutables y documentación para impedir rutas personales conocidas.
- Tests de contrato verifican que los mapas se incluyan en `setup.py` y que los launch/map loader resuelvan el share del paquete.
- Se ejecutan los tests existentes disponibles y compilación sintáctica Python.
- Un entorno ROS completo deberá ejecutar `colcon build` y los launch como validación final de runtime.
