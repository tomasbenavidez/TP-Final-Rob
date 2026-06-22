# Diseño — HTML didáctico de la arquitectura completa

## Objetivo

Rediseñar `docs/arquitectura_sistema_completo.html` para explicar el TP como un
sistema integral. La página debe servir simultáneamente como introducción para
una persona nueva, apoyo para una defensa oral y referencia técnica de nodos,
tópicos, TF y artefactos.

La idea central será cómo el robot transforma información en movimiento:

**Percibir → Estimar → Decidir → Actuar**

## Alcance

- Parte A implementada: dos pasadas separadas sobre el rosbag real para estimar
  la trayectoria con Graph SLAM y producir la grilla de ocupación.
- Parte B implementada: uso del mapa en Gazebo para localización MCL,
  planificación, seguimiento y evasión.
- Parte C futura: cierre conceptual sobre despliegue en el robot real y
  percepción activa. Debe aparecer siempre como **no implementada**, sin
  atribuirle nodos, tópicos ni contratos todavía inexistentes.
- El entregable seguirá siendo un único HTML autocontenido, sin dependencias de
  red ni proceso de build.

## Arquitectura de la información

La portada presentará el ciclo conceptual y un estado breve del trabajo:
Partes A y B implementadas; Parte C futura. No mostrará referencias a ramas o
commits, porque se vuelven obsoletas y no ayudan a comprender el sistema.

El contenido principal será un mapa panorámico interactivo con tres franjas
secuenciales:

1. **Parte A — construir conocimiento:** rosbag → percepción ArUco y odometría
   → Graph SLAM → trayectoria corregida → proyección LIDAR → mapa.
2. **Parte B — usar el conocimiento:** mapa y sensores simulados → MCL → A* y
   máquina de estados → control del movimiento.
3. **Parte C — llevarlo al robot real:** intención futura, diferenciada por
   color, tramado y la etiqueta visible `NO IMPLEMENTADO`.

La transición A → B mostrará que el único artefacto persistente compartido es
`map.yaml` + `map.pgm`. También explicará que las etapas se ejecutan por
separado.

## Modos de lectura

Un selector alternará la misma geometría del mapa entre dos niveles, sin mover
al lector a otra página ni duplicar el diagrama:

- **Entender el sistema:** capacidades, preguntas que responde cada etapa y
  explicaciones en lenguaje llano.
- **Arquitectura ROS:** nombres exactos de nodos, tópicos, TF, frames, archivos
  y tipos relevantes.

Al cambiar de modo se conservarán la selección y el contexto visual actuales.

## Interacción didáctica

Cada bloque tendrá tres niveles de profundidad:

1. Tarjeta breve con nombre y función.
2. Selección que resalta entradas, salidas y conexiones relacionadas y atenúa
   el resto del mapa.
3. Panel de detalle que responde: qué problema resuelve, qué recibe, qué
   produce, qué nodo lo implementa, por qué es necesario y qué ocurriría si
   faltara.

Habrá cuatro recorridos guiados que resaltarán secuencias sobre el mismo mapa:

- Cómo se construye el mapa.
- Cómo sabe el robot dónde está.
- Cómo llega al objetivo.
- Qué ocurre ante un obstáculo nuevo.

Los recorridos permitirán avanzar, retroceder y salir sin dejar la interfaz en
un estado inconsistente.

## Contratos y advertencias contextuales

Las siguientes diferencias se mostrarán cerca del bloque afectado, no sólo en
una lista final:

- Parte A y Parte B no se lanzan simultáneamente.
- `graph_slam_node` y `mcl_localization` son productores alternativos de
  `map → odom`.
- `/landmarks` es `MarkerArray` de ArUco optimizados en A y `PoseArray` de
  referencias virtuales conocidas en B.
- Parte A usa `tb4_0/*` y `base_link`; Parte B usa `/scan`, `/odom` y
  `base_footprint`.
- `landmark_sensor` usa `odom` como verdad simulada para evitar realimentar MCL.
- `state_machine` es el único productor de `/cmd_vel` dentro de Parte B.
- La segunda pasada de A compone la corrección SLAM con odometría densa para
  proyectar el LIDAR; no interpola simplemente entre keyframes.

## Estilo visual y accesibilidad

- Estética de plano técnico claro y didáctico.
- Parte A en turquesa, Parte B en naranja y Parte C en gris/violeta tramado.
- Tópicos con línea continua, TF con línea discontinua y archivos con línea
  punteada.
- Estados que no dependan sólo del color; usar texto, iconos o patrones.
- Controles semánticos, navegación por teclado, foco visible y regiones vivas
  para cambios importantes.
- Animación breve sólo para orientar la atención y desactivada con
  `prefers-reduced-motion`.
- En escritorio se conservará el panorama. En pantallas angostas el mismo flujo
  se reordenará verticalmente, sin exigir desplazamiento horizontal para leerlo.

## Fuente de verdad del contenido

Los textos y conexiones se contrastarán con los launch files, publishers,
subscribers y documentación vigente del repositorio. No se cambiarán algoritmos
ni parámetros de SLAM, MCL, A*, pure pursuit o evasión como parte de esta tarea.

## Verificación

- Validar la sintaxis del HTML y que el JavaScript cargue sin errores.
- Probar ambos modos conservando la selección.
- Probar selección/deselección de bloques y los cuatro recorridos guiados.
- Verificar contratos técnicos contra el código actual.
- Revisar navegación por teclado, foco visible y reducción de movimiento.
- Inspeccionar visualmente tamaños de escritorio y móvil en el navegador.
- Ejecutar `git diff --check` y las verificaciones portables indicadas por
  `AGENTS.md` para confirmar que el cambio documental no introduce regresiones.

## Fuera de alcance

- Implementar la Parte C.
- Modificar nodos, algoritmos, parámetros o launch files.
- Unificar contratos válidamente diferentes entre las Partes A y B.
- Agregar frameworks, paquetes de JavaScript, servicios externos o telemetría.
