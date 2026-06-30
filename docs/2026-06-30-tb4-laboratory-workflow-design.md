# Diseño del flujo integral de laboratorio con TurtleBot4

**Fecha:** 2026-06-30

**Estado:** propuesta lista para revisión

**Alcance:** arquitectura, validaciones, seguridad, portabilidad TB3/TB4 y guía operativa futura

## 1. Objetivo

El sistema debe permitir completar en un laberinto real, con un TurtleBot4, el
ciclo entero del TP:

1. recorrer manualmente el laberinto y grabar una única adquisición;
2. reproducir esa adquisición para estimar trayectoria y landmarks mediante
   Graph SLAM;
3. reproducirla nuevamente para generar la grilla de ocupación;
4. usar la grilla y los landmarks obtenidos para localizar y navegar el mismo
   TurtleBot4 hacia objetivos elegidos por el usuario;
5. agregar conos después del mapeo y ejecutar una misión autónoma que encuentre
   el cono rojo, ignore los distractores, evite obstáculos y no continúe
   moviéndose si pierde localización o datos sensoriales críticos.

El flujo debe ser reproducible con cualquiera de los robots del laboratorio
identificados mediante los namespaces `tb4_0` y `tb4_1`.

## 2. Decisiones confirmadas

### 2.1 Topología de ejecución

El sistema se distribuye de la siguiente manera:

**TurtleBot4**

- drivers de la base;
- drivers del LIDAR y la cámara;
- publicación de odometría y TF;
- mecanismos de seguridad propios del robot;
- grabación del rosbag sobre almacenamiento local del robot.

**Notebook Linux del laboratorio**

- checkout y build de este repositorio;
- reproducción de rosbags;
- detección ArUco;
- Graph SLAM con GTSAM;
- generación de la grilla;
- MCL;
- planificación global;
- control y máquina de estados;
- monitor de obstáculos dinámicos;
- percepción de conos;
- supervisor de misión;
- RViz, diagnósticos y conservación de resultados.

La notebook no inicia una segunda copia de los drivers del robot. El TB4 no
ejecuta los algoritmos pesados del TP.

### 2.2 Adquisición única para Parte A

El robot recorre físicamente el laberinto una sola vez. Esa adquisición se
graba en un rosbag dentro del TB4. Después se copia completa a la notebook y se
reproduce dos veces:

```text
adquisición real única
        |
        +-- reproducción 1 --> trayectoria y landmarks optimizados
        |
        `-- reproducción 2 --> grilla de ocupación
```

No se intenta repetir físicamente la misma trayectoria. Las dos pasadas deben
consumir el mismo bag para conservar mediciones y timestamps idénticos.

### 2.3 Separación de perfiles

No se utiliza un único booleano `is_tb4`. Se definen tres perfiles de ejecución:

- `simulation_tb3`: Gazebo, TurtleBot3, landmarks virtuales y tiempo simulado;
- `bag_tb4`: reproducción offline, tópicos TB4 y tiempo simulado publicado por
  `ros2 bag play --clock`;
- `real_tb4`: sensores y base reales, reloj real y landmarks ArUco reales.

Los algoritmos permanecen compartidos. Los perfiles sólo seleccionan tópicos,
frames, fuentes de datos, parámetros físicos y nodos habilitados.

## 3. Alcance de los cambios futuros

### 3.1 Renombres de paquetes

Se realizarán, como cambios puramente estructurales:

- el paquete heredado de interfaces de SLAM → `tp_interfaces`;
- el paquete heredado de SLAM de Parte A → `tp_a_slam_aruco`.

El primer paquete seguirá separado porque contiene mensajes compartidos por los
productores de observaciones reales y MCL. El nombre nuevo refleja que no son
interfaces exclusivas de SLAM.

Los renombres no modificarán algoritmos, parámetros ni contratos ROS. Se
ejecutarán después de congelar una línea base y antes de introducir los perfiles
TB3/TB4, para que cualquier regresión estructural pueda aislarse.

El mensaje `VisualObservability` deberá clasificarse durante el renombre:

- se conserva si se conecta a diagnósticos runtime;
- se elimina si continúa usado únicamente por tests y utilidades desconectadas.

La decisión se toma a partir de referencias runtime, no por el nombre del
mensaje.

### 3.2 Contrato común de plataforma

Cada perfil deberá resolver explícitamente:

| Campo | Ejemplo TB3 simulado | Ejemplo TB4 real |
|---|---|---|
| `robot_namespace` | vacío | `/tb4_0` o `/tb4_1` |
| `odom_topic` | `/calc_odom` | `<ns>/odom` |
| `reference_odom_topic` | `/odom` | `<ns>/odom` |
| `scan_topic` | `/scan` | `<ns>/scan` |
| `rgb_topic` | cámara Gazebo | tópico OAK-D descubierto |
| `depth_topic` | depth Gazebo | depth métrico alineado con RGB |
| `camera_info_topic` | cámara Gazebo | `CameraInfo` de la OAK-D |
| `cmd_vel_topic` | `/cmd_vel` | `<ns>/cmd_vel` |
| `base_frame` | `base_footprint` | frame publicado por el TB4 |
| `odom_frame` | `odom` | frame publicado por el TB4 |
| `lidar_frame` | `scan.header.frame_id` | `scan.header.frame_id` |
| `use_sim_time` | `true` | `false` |
| landmarks | virtuales | ArUco identificados |

Los nombres de frames no se construyen concatenando el namespace sin verificar.
Se obtienen de los mensajes y del árbol TF o se pasan como parámetros validados.

### 3.3 LIDAR independiente de la plataforma

Todo consumidor del LIDAR debe respetar:

- `scan.header.frame_id`;
- `angle_min`, `angle_max` y `angle_increment`;
- rangos no finitos o fuera de los límites;
- sectores ocluidos o sin retornos;
- transformación TF desde el frame del LIDAR al frame base;
- timestamp del scan.

No se debe asumir que el ángulo cero del LIDAR coincide con el frente del robot.
Tampoco se debe asumir cobertura útil de 360 grados aunque el sensor sea
nominalmente panorámico.

Una zona no observada no se marca como libre. Parte A no dependerá de
`lidar_yaw=π/2` y `lidar_tx=-0.04` salvo como fallback documentado por perfil.
Parte B no proyectará retornos del scan como si ya estuvieran expresados en
`base_link`.

## 4. Organización de corridas y artefactos

Cada sesión se identifica mediante un `run_id` estable:

```text
YYYYMMDD-HHMM-<robot>
```

Ejemplo:

```text
20260630-1430-tb4_1
```

Los artefactos de una sesión se conservan juntos fuera del checkout o bajo una
carpeta de resultados explícitamente ignorada por Git:

```text
runs/<run_id>/
├── acquisition/
│   └── <rosbag completo>
├── config/
│   ├── platform-resolved.yaml
│   └── bag-topics.txt
├── parte_a/
│   ├── trajectory.json
│   ├── map.pgm
│   ├── map.yaml
│   ├── aruco_detections.csv
│   └── aruco_geometry_debug.csv
├── diagnostics/
│   ├── trajectory-comparison.*
│   ├── map-quality.*
│   └── mcl-obstacle-run/
└── logs/
```

Los resultados no se escriben dentro de `share/` ni dependen de rutas
personales. El `run_id`, el namespace y los parámetros resueltos deben permitir
repetir el procesamiento más tarde.

## 5. Preflight del laboratorio

Antes de mover el robot se deben comprobar estas condiciones.

### 5.1 Identidad y red

- confirmar si el robot es `tb4_0` o `tb4_1`;
- confirmar que notebook y TB4 se descubren mediante la configuración ROS 2
  disponible en el laboratorio;
- confirmar `ROS_DOMAIN_ID`, RMW y discovery server usados;
- listar tópicos del namespace elegido;
- verificar que no se ven simultáneamente tópicos ambiguos de ambos robots;
- comprobar acceso SSH y espacio libre suficiente en el TB4;
- comprobar un mecanismo confiable para copiar el bag a la notebook.

### 5.2 Sensores y TF

- recibir odometría;
- recibir LIDAR con frecuencia estable;
- recibir RGB y `CameraInfo`;
- identificar el tópico de depth métrico alineado con RGB;
- verificar TF `odom → base`;
- verificar TF `base → lidar`;
- verificar TF `base → camera`;
- comprobar timestamps recientes y coherentes;
- visualizar scan, imagen y modelo del robot en RViz antes de grabar.

### 5.3 Seguridad y movimiento

- identificar el e-stop o método de detención inmediata;
- designar una persona responsable de detener el robot;
- confirmar que sólo existe un productor normal de `cmd_vel`;
- mantener apagados `state_machine` y la misión durante teleoperación/mapeo;
- comenzar con límites de velocidad reducidos;
- verificar que al cerrar el nodo de control el robot recibe velocidad cero;
- mantener libre una zona de prueba corta antes de entrar al laberinto.

## 6. Contrato del rosbag

El bag de Parte A debe contener, bajo el namespace seleccionado:

- odometría;
- scan;
- imagen RGB usada por ArUco;
- `CameraInfo` correspondiente;
- TF dinámico;
- TF estático.

Para diagnóstico y reutilización se recomienda registrar además, cuando estén
disponibles:

- IMU;
- wheel ticks o wheel states;
- joint states;
- hazard/bump/cliff detection;
- imagen depth alineada;
- información diagnóstica de los drivers.

El grabador se ejecuta en el TB4. La teleoperación puede originarse en la
notebook, pero ninguna imagen de alta tasa depende de llegar a la notebook para
quedar almacenada.

Al detener la grabación se valida:

- que `metadata.yaml` existe;
- que el bag abre con `ros2 bag info`;
- que todos los tópicos obligatorios están presentes;
- que la duración coincide con el recorrido;
- que cada tópico crítico tiene una cantidad de mensajes razonable;
- que el bag puede reproducirse desde la notebook;
- que una muestra de imagen, scan, odometría y TF se visualiza correctamente.

El verificador debe aceptar un namespace como entrada; no debe contener
`/tb4_0` como contrato fijo.

## 7. Parte A: procesamiento y gates de aceptación

### 7.1 Primera pasada

La primera reproducción ejecuta:

- puente TF sólo si el bag usa tópicos TF namespaced no consumibles directamente;
- detección ArUco;
- transformación cámara → base;
- construcción y optimización del grafo;
- publicación de trayectoria y landmarks;
- exportación de JSON y diagnósticos.

Se usa `--clock` y todos los nodos offline usan `use_sim_time=true`.

### 7.2 Gate A1: calidad visual y de landmarks

Antes de generar la grilla:

- existe calibración proveniente de `CameraInfo` o fallback explícito del robot;
- las detecciones transformadas quedan delante/lateralmente en posiciones
  razonables;
- los IDs corresponden a los tags del escenario;
- existe una cantidad suficiente de IDs reobservados;
- los cierres de lazo no dependen de una única detección aislada;
- los residuos y saltos espaciales no muestran una población grande de
  outliers aceptados;
- se conoce qué porcentaje de observaciones usó TF real y cuál fallback.

### 7.3 Gate A2: calidad de trayectoria

Se compara trayectoria optimizada con odometría cruda:

- la corrección global reduce deriva;
- la trayectoria optimizada es continua;
- no aparecen saltos instantáneos;
- `map ← odom` cambia suavemente;
- los cierres de lazo producen correcciones explicables;
- los keyframes y observaciones están temporalmente asociados.

Si el gate falla, no se ajustan log-odds ni se genera un mapa presentado como
válido.

### 7.4 Segunda pasada y diagnóstico del mapa grueso

La segunda reproducción integra LIDAR usando trayectoria corregida y odometría
densa.

El diagnóstico se ejecuta en este orden:

1. generar el mapa actual como baseline;
2. regenerar descartando scans durante rotaciones rápidas;
3. comparar espesor de paredes y continuidad;
4. verificar extrínsecos contra TF;
5. comprobar que cada scan tenga odometría anterior y posterior;
6. medir el efecto de usar una única pose para todo el scan;
7. determinar si es necesario deskew por rayo;
8. sólo después evaluar log-odds y resolución.

Los cambios se prueban de a una variable usando el mismo bag.

### 7.5 Conservación de objetos finos

La desaparición de patas de silla se estudia después de estabilizar la pose del
scan. Se comparan:

- cantidad de hits y misses por celda;
- relación entre evidencia ocupada y libre;
- resolución de 5 cm frente a una resolución más fina;
- evidencia necesaria para limpiar una celda previamente ocupada;
- vecindad espacial del endpoint.

No se usa dilatación global para conservar patas si eso engrosa paredes. Tampoco
se eleva el umbral de ocupación como primera medida, porque puede borrar antes
los objetos finos.

### 7.6 Gate A3: mapa navegable

El mapa se acepta para B/C sólo si:

- representa la geometría completa del laberinto;
- no tiene paredes dobles o abanicos severos;
- conserva aperturas transitables;
- no crea conexiones a través de paredes;
- A* encuentra rutas entre un conjunto de pares distribuidos;
- las rutas mantienen clearance suficiente para el TB4;
- landmarks y grilla siguen expresados en el mismo `map`;
- el JSON y el YAML pertenecen al mismo `run_id`.

## 8. Parte B real

### 8.1 Pila esperada

El perfil `real_tb4` ejecuta en la notebook:

- `map_loader`;
- `mcl_localization` con el JSON de landmarks;
- `global_planner`;
- `obstacle_monitor`;
- `state_machine`;
- detector ArUco;
- adaptador ArUco → observaciones identificadas;
- RViz y diagnósticos.

No ejecuta:

- `landmark_publisher`;
- `landmark_sensor`;
- `/calc_odom` de simulación;
- drivers duplicados del TB4;
- Graph SLAM como productor simultáneo de `map → odom`.

MCL es el único productor de `map → odom`.

### 8.2 Gate B1: localización sin movimiento autónomo

- cargar mapa y JSON de la misma corrida;
- publicar una `/initialpose` razonable;
- comprobar una nube inicialmente concentrada alrededor de esa pose;
- observar correcciones al ver ArUco reales;
- verificar que la pose estimada coincide visualmente con scan y mapa;
- comprobar covarianza acotada;
- confirmar que la nube no es multimodal;
- comprobar que no se publican comandos de velocidad.

MCL no usa actualmente la grilla para invalidar partículas dentro de paredes.
Este límite debe quedar visible en los diagnósticos y criterios de seguridad.

### 8.3 Gate B2: navegación básica

- usar velocidad reducida;
- elegir un objetivo cercano, libre y con camino despejado;
- verificar plan, movimiento y orientación final;
- repetir con objetivos progresivamente más lejanos;
- probar un objetivo nuevo durante el recorrido;
- medir error final y distancia mínima a paredes;
- comprobar que el simplificador de ruta no anula el clearance producido por
  A*.

`clearance_weight` no se modifica mientras los caminos sean seguros, suaves y
transitables. Cualquier cambio requiere una métrica que muestre la deficiencia.

### 8.4 Diagnóstico de evasión

Antes de modificar MCL o el monitor se reproduce una corrida problemática y se
registra:

- `/calc_odom` y `/odom` en simulación;
- `/particlecloud`;
- `/mcl_pose` y covarianza;
- cantidad de landmarks visibles;
- `/obstacle_detected`;
- `/dynamic_obstacles`;
- `/nav_state`;
- `/cmd_vel`;
- TF `map → odom → base`;
- timestamps del primer falso obstáculo y del inicio de divergencia.

La investigación debe responder:

1. ¿MCL diverge antes de que aparezca el falso obstáculo?
2. ¿El obstáculo causa pérdida de observabilidad antes de la divergencia?
3. ¿La odometría de movimiento coincide con el movimiento ejecutado?
4. ¿La nube se vuelve multimodal y la media cae entre modos?
5. ¿Una pared conocida se proyecta sobre una celda libre por usar una pose
   incorrecta?

No se ajustan simultáneamente ruido de MCL, TTL, inflación y maniobra de evasión.

### 8.5 Gate B3: obstáculos dinámicos

La validación ocurre en incrementos:

1. detección y visualización sin evasión autónoma;
2. inserción de capa dinámica con robot detenido;
3. replanning sin movimiento;
4. evasión a velocidad reducida;
5. recuperación de localización después de la maniobra;
6. recorrido completo con obstáculo.

El robot debe detenerse o rechazar inserciones globales cuando:

- la pose MCL está vencida;
- la covarianza supera el límite aceptado;
- el scan está vencido;
- no existe TF al timestamp relevante.

La detección reactiva en el frame del robot puede seguir indicando peligro sin
una pose global confiable; lo que se suspende es la proyección sobre el mapa.

## 9. Parte C real

### 9.1 Preparación del escenario

- se utiliza el mismo laberinto de Parte A/B;
- paredes, ArUco e IDs no se mueven después del mapeo;
- los conos se agregan después de generar el mapa;
- se colocan distractores de otros colores;
- se incluye al menos una ubicación que obligue a distinguir visibilidad de
  alcanzabilidad sin bloquear todas las rutas.

### 9.2 Gate C1: percepción

Antes de habilitar movimiento autónomo:

- confirmar RGB;
- confirmar depth métrico alineado;
- comprobar sincronización RGB-depth;
- verificar TF cámara → map;
- ensayar rojo y distractores;
- validar confirmación temporal;
- comprobar pose del cono en RViz;
- comprobar que la pose y covarianza cambian razonablemente con la distancia.

### 9.3 Gate C2: aproximación

- validar pose de aproximación sobre la grilla;
- confirmar línea de visión desde la pose final;
- exigir camino A* alcanzable;
- tratar conos no rojos como obstáculos dinámicos;
- eximir el cono rojo confirmado sólo dentro del radio necesario;
- conservar una distancia final segura.

### 9.4 Gate C3: misión completa

- misión inicia sólo con mapa, MCL y visión frescos;
- explora sin repetir indefinidamente objetivos agotados;
- no atraviesa paredes aunque vea el cono;
- se detiene ante pérdida de localización, scan o visión;
- alcanza el cono rojo y publica `FOUND`;
- ignora los conos de otros colores;
- permite cancelación y parada manual inmediata.

## 10. Seguridad runtime requerida

Antes de declarar el perfil real listo deben existir políticas explícitas para:

- covarianza MCL excesiva;
- TF vencido;
- odometría vencida;
- scan vencido;
- cámara vencida durante Parte C;
- ausencia prolongada de observaciones de landmarks;
- plan fallido;
- controlador o monitor caído;
- recepción de un cancel;
- intervención manual.

La ausencia de mensajes no equivale a un valor seguro. Por ejemplo, si el
monitor deja de publicar, la FSM no debe inferir automáticamente que no hay
obstáculo.

El runbook debe incluir el procedimiento de parada independientemente de que
los nodos ROS respondan.

## 11. Robustez para `tb4_0` y `tb4_1`

La selección del robot se realiza una vez al comenzar la sesión. Esa selección
alimenta todos los launch y verificadores.

El sistema debe:

- derivar tópicos a partir del namespace;
- aceptar excepciones mediante remaps explícitos;
- leer intrínsecos desde `CameraInfo`;
- leer extrínsecos desde TF;
- usar archivos de fallback identificados por robot sólo cuando TF o
  `CameraInfo` no estén disponibles;
- registrar en los resultados qué fuente se utilizó;
- evitar archivos de salida compartidos entre robots;
- validar que ningún tópico crítico quedó apuntando al otro namespace.

El comportamiento no se considera robusto sólo porque `/tb4_0` pueda
reemplazarse textualmente por `/tb4_1`.

## 12. Estrategia documental

El README raíz se reescribirá como guía ejecutable del día de laboratorio:

1. preparación de notebook y robot;
2. selección del namespace;
3. preflight;
4. seguridad;
5. grabación onboard;
6. copia y validación del bag;
7. primera pasada de Parte A;
8. segunda pasada de Parte A;
9. inspección y aceptación del mapa;
10. Parte B real;
11. diagnóstico de localización;
12. navegación básica;
13. obstáculos dinámicos;
14. colocación de conos;
15. Parte C;
16. cancelación, recuperación y troubleshooting;
17. artefactos que deben conservarse.

El README tendrá comandos completos y orden de terminales. Los fundamentos,
gráficos de diagnóstico y problemas conocidos permanecerán en documentos
específicos bajo `docs/`.

La guía no mezclará comandos de simulación con el procedimiento real. Cada
bloque indicará claramente dónde se ejecuta:

- `[TB4 por SSH]`;
- `[Notebook laboratorio]`;
- `[RViz]`;
- `[Acción física]`.

## 13. Descomposición en planes futuros

Este diseño se implementará mediante planes separados y verificables:

1. línea base, artefactos y verificadores de bag;
2. renombre `tp_interfaces`;
3. renombre `tp_a_slam_aruco`;
4. contrato de plataforma y namespaces;
5. transformación LIDAR basada en TF;
6. diagnóstico y mejora del mapa de Parte A;
7. diagnóstico MCL/obstáculos;
8. perfil Parte B real y seguridad runtime;
9. perfil Parte C real y contrato RGB-depth;
10. README y runbook final;
11. ensayo integral de laboratorio.

Ningún plan de corrección algorítmica se redactará como definitivo antes de
obtener la evidencia diagnóstica correspondiente.

## 14. Criterio de éxito integral

El objetivo se considera cumplido cuando, usando uno de los TB4 del laboratorio:

1. se graba una sola adquisición onboard;
2. el bag supera el contrato y puede reproducirse en la notebook;
3. las dos pasadas producen trayectoria, landmarks y mapa navegable;
4. MCL localiza el robot real en ese mapa;
5. el robot alcanza múltiples objetivos manuales sin colisiones;
6. detecta y rodea un obstáculo agregado sin perder localización;
7. encuentra el cono rojo entre distractores;
8. no intenta atravesar paredes;
9. se detiene de manera segura ante pérdida de datos o localización;
10. todos los comandos necesarios están documentados y pueden seguirse desde
    una sesión limpia en la notebook del laboratorio.
