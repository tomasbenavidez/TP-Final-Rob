Universidad de San Andr´es, Depto. de Ingenier´ıa 

## Trabajo Pr´actico Final 


I-402 - Principios de la Rob´otica Aut´onoma 

## Prof. Ignacio Mas, Tadeo Casiraghi y Bautista Chasco 

En este trabajo pr´actico final, los alumnos deber´an integrar los principales conceptos de los enfoques probabil´ısticos abordados a lo largo de la materia mediante la implementaci´on de un sistema aut´onomo de localizaci´on y mapeo simult´aneos (SLAM). 

Utilizando un robot TurtleBot3 simulado en el entorno de Gazebo, o bien un RosBag del TurtleBot4, el objetivo en esta primera etapa (Parte A) ser´a que el robot explore un entorno desconocido de tipo laberinto y construya una representaci´on (mapa) del mismo mientras estima su propia posici´on. 

Esta fase representa un caso de aplicaci´on realista donde convergen m´ultiples herramientas estudiadas durante la cursada, tales como la estimaci´on de estado en presencia de ruido, el procesamiento de sensores inexactos (como LIDAR o c´amaras) y la fusi´on de informaci´on sensorial. La correcta implementaci´on de esta etapa es fundamental, dado que el mapa generado y la precisi´on de la localizaci´on ser´an el pilar sobre el cual se desarrollar´an las Partes B y C, centradas en la navegaci´on aut´onoma. 

1 

## **SLAM - Generaci´on del mapa** 

En esta primera etapa, los alumnos deber´an implementar un sistema de SLAM utilizando el robot TurtleBot3 en un entorno simulado en Gazebo o el TurtleBot4 con datos pregrabados. El objetivo principal es que la plataforma explore un escenario tipo laberinto y construya un mapa de ocupaci´on empleando las lecturas de los sensores y la odometr´ıa. 

Esta etapa permite aplicar de forma pr´actica los algoritmos de estimaci´on de estado y mapeo estudiados en la materia, ya sea mediante t´ecnicas de SLAM basadas en Filtros de Part´ıculas (FastSLAM), Filtros de Kalman Extendidos (EKF SLAM) o cualquier otro m´etodo pertinente. Al finalizar esta secci´on, se deber´a exportar el mapa generado y verificar su calidad, ya que servir´a como base fundamental para la navegaci´on en la siguiente fase del trabajo. 

Para el desarrollo de esta primera parte, deber´an elegir uno de los 3 caminos posibles de implementaci´on. 

## **1. Opci´on 1: Grid-Based FastSLAM** 

|**Entorno de Trabajo**|Gazebo|
|---|---|
|**Sensores**|LIDAR|
|**Algoritmos**|Likelihood Fields<br>Modelo de Odometr´ıa<br>Filtro de Part´ıculas<br>Occupancy Grids<br>Grid-Based FastSLAM|



2 

Para este camino deber´an implementar el algoritmo Grid-Based FastSLAM en el entorno simulado de Gazebo. Se ver´an forzados a optimizar el c´odigo para que no solo se ejecute de manera eficiente en sus equipos, sino que tambi´en permita procesar una cantidad razonable de part´ıculas en tiempo real. Para cada part´ıcula, deber´an mantener un mapa estimado mediante _likelihood fields_ para lograr una localizaci´on efectiva mientras realizan el mapeo. 

## **1.1. Preparaci´on del entorno** 

Antes de comenzar con la implementaci´on de SLAM, es necesario configurar correctamente el entorno de simulaci´on. Para esto, se debe contar con una instalaci´on funcional de ROS 2 Humble, Gazebo y los paquetes de TurtleBot3, ya sea de forma nativa en Ubuntu 22.04 (o cualquier sistema operativo compatible) o mediante Robostack. Recuerden que disponen de las computadoras del laboratorio de inform´atica en el edificio Sullair; no obstante, si optan por esta alternativa, act´uen con cautela para no dejar el c´odigo desarrollado expuesto en equipos p´ublicos. 

## **1.2. Lanzamiento del robot y teleoperaci´on** 

Una vez configurado el entorno, el siguiente paso es lanzar el robot TurtleBot3 dentro del laberinto simulado para explorar el terreno. Para ello, ejecuten los siguientes comandos en dos terminales independientes: 

- **Terminal 1:** `ros2 launch turtlebot3_custom_simulation custom_casa.launch.py` 

- **Terminal 2:** `ros2 run turtlebot3_teleop teleop_keyboard` 

## **1.3. Odometr´ıa** 

La odometr´ıa requerida es la publicada en el t´opico `calc_odom` . Esta representa la posici´on estimada del robot, por lo que deben contemplar que acumular´a un error considerable a lo largo del tiempo. Si buscan actualizar la odometr´ıa utilizando el modelo de deltas ( _δθ_ 1 _, δθ_ 2 _, δtrans_ ), recuerden calcular la diferencia respecto a la odometr´ıa del instante de tiempo anterior. 

## **1.4. SLAM en ROS 2** 

En esta etapa, dise˜nar´an e implementar´an su propio algoritmo de SLAM utilizando las lecturas del TurtleBot3. El prop´osito es construir el mapa del entorno al mismo tiempo que se estima la pose del robot. Al finalizar, guarden obligatoriamente este mapa, ya que ser´a el insumo de localizaci´on para las partes B y C del TP Final. 

## **1.5. Visualizaci´on con RViz** 

Durante la ejecuci´on del algoritmo, se recomienda fehacientemente el uso de RViz para monitorear las se˜nales de entrada y la salida del sistema. Los elementos clave a visualizar son: 

3 

- `/scan` : Visualizaci´on de los rayos del LIDAR. 

- `/odom` : Posici´on real del robot ( _ground truth_ ). 

- `/calc_odom` : Posici´on estimada por odometr´ıa pura. 

- `/belief` : Pose corregida del robot (estimaci´on de SLAM). 

- `/map` o `/likelihoodfield` : Mapa de ocupaci´on generado en tiempo real. 

Pueden cargar un archivo `.rviz` preconfigurado o a˜nadir los elementos manualmente en la interfaz. 

## **1.6. Mapa final** 

El entregable final de esta opci´on consistir´a en un mapa de grilla de ocupaci´on optimizado, el cual ser´a exportado formalmente para su uso posterior en las etapas de navegaci´on. 

4 

## **2. Opci´on 2: Features con LIDAR** 

|**Entorno de Trabajo**|Gazebo|
|---|---|
|**Sensores**|LIDAR|
|**Algoritmos**|Extracci´on de caracter´ısticas (features) mediante LIDAR<br>Modelo de Odometr´ıa<br>Algoritmos de SLAM basados en caracter´ısticas, p. ej.:<br>- EKF SLAM<br>- Graph SLAM<br>- SEIF SLAM|



Para este camino, deber´an implementar un m´odulo de detecci´on y extracci´on de caracter´ısticas geom´etricas ( _features_ ) a partir de los datos crudos del LIDAR. Tengan en cuenta que estos puntos de referencia deben ser razonablemente estables y repetibles ante cambios de perspectiva para actuar como hitos ( _landmarks_ ) confiables. 

Una vez resuelta la extracci´on, podr´an aplicar el algoritmo de SLAM basado en caracter´ısticas de su preferencia. En este caso, el proceso de mapeo es doble: 

1. **Primera pasada:** Construcci´on y consolidaci´on del mapa de caracter´ısticas puntuales para resolver la localizaci´on del robot. 

2. **Segunda pasada:** Con la localizaci´on ya resuelta y corregida por el mapa de caracter´ısticas, realizar´an un mapeo de grilla de ocupaci´on cl´asico con el LIDAR. Esto simplifica el proceso, ya que no deber´an preocuparse por la distorsi´on del mapa derivada del error de trayectoria. 

Este mapa de grillas definitivo es estrictamente necesario para que el robot pueda planificar trayectorias fluidas sin colisionar con los obst´aculos en las Partes B y C. 

## **2.1. Preparaci´on del entorno** 

El entorno operativo replica las especificaciones de la secci´on anterior: requiere ROS 2 Humble, Gazebo y las dependencias de TurtleBot3 configuradas en sus entornos locales o en las terminales asignadas en el laboratorio Sullair. 

## **2.2. Lanzamiento del robot y teleoperaci´on** 

El despliegue de la simulaci´on y el nodo de teleoperaci´on manual se efect´ua mediante los mismos comandos de lanzamiento de la opci´on previa. 

5 

## **2.3. Odometr´ıa** 

Al igual que en la opci´on anterior, se deber´a consumir el t´opico `calc_odom` y procesar de forma adecuada el modelo de deltas cinem´aticos para la actualizaci´on predictiva del filtro o grafo elegido. 

## **2.4. SLAM en ROS 2** 

Dise˜nen el nodo de SLAM para estimar simult´aneamente las posiciones del robot y las coordenadas de los hitos detectados. Al concluir, el sistema debe ser capaz de guardar las estructuras de datos correspondientes a la infraestructura del mapa. 

## **2.5. Visualizaci´on con RViz** 

- Monitoreen el desempe˜no del estimador en RViz prestando especial atenci´on a: 

- `/scan` : Puntos del LIDAR e hitos detectados superpuestos. 

- `/odom` y `/calc_odom` : Comparativa de trayectorias. 

- `/belief` : Pose corregida bajo el mapa de caracter´ısticas. 

- `/landmarks` o `/map` : Representaci´on de los hitos y la grilla de ocupaci´on seg´un la pasada actual. 

## **2.6. Mapa final** 

El resultado esperado obligatorio para esta opci´on consta del mapa de grilla de ocupaci´on del entorno y, de manera complementaria, el mapa geom´etrico con la ubicaci´on de las caracter´ısticas (landmarks) validadas. 

6 

## **3. Opci´on 3: Features con C´amara** 

|**Entorno de Trabajo**|RosBag (Datos reales de TurtleBot4)|
|---|---|
|**Sensores**|LIDAR, C´amara|
|**Algoritmos**|Detecci´on y extracci´on de caracter´ısticas v´ıa ArUco Tags<br>Modelo de Odometr´ıa<br>Graph SLAM (Obligatorio)|



Debido a que el entorno est´andar de Gazebo no modela con suficiente fidelidad los efectos ´opticos de una c´amara real, para esta opci´on trabajar´an con un archivo de registro de datos ( _RosBag_ ). Un _RosBag_ es una grabaci´on temporal de los t´opicos y mensajes del sistema; al reproducirlo, sus nodos locales recibir´an la informaci´on de la c´amara y el LIDAR exactamente como si la plataforma f´ısica estuviera operando en vivo. 

Deber´an emplear este registro para dise˜nar un algoritmo robusto de detecci´on de marcadores ArUco. Esto implica mitigar problemas reales como el desenfoque por movimiento ( _motion blur_ ) y la baja densidad de los tags en ciertas zonas del laberinto. Para esta opci´on, **es obligatorio implementar Graph SLAM** , incorporando mecanismos de cierre de lazo ( _loop closure_ ) para optimizar el grafo global de poses. 

Al igual que en la opci´on anterior, el procedimiento requiere dos etapas: una primera aproximaci´on para consolidar el grafo de poses y landmarks visuales, y una segunda reproducci´on con la trayectoria corregida para proyectar las lecturas de LIDAR en una grilla de ocupaci´on consistente. 

## **3.1. Preparaci´on del entorno** 

En este escenario no se ejecutar´a Gazebo. Deber´an descargar los paquetes de datos, la matriz de calibraci´on intr´ınseca de la c´amara ( _K_ ) y los coeficientes de la c´amara desde el siguiente ENLACE. Los archivos incluidos son: 

- `aruco_estimation.zip` : Grabaci´on corta con un marcador ArUco dispuesto a diferentes distancias controladas, ideal para calibrar y caracterizar el modelo de medici´on. 

- `laberinto.zip` : Registro extenso que recorre el laberinto completo realizando m´ultiples bucles. Se utilizar´a para la generaci´on del mapa final. 

- `Matrices, coeficientes y estimaciones` : Archivo con los datos de las camaras utilizadas. El RosBag fue grabado con el Turtlebot 4 n´umero 0. 

## **3.2. Lanzamiento del robot y teleoperaci´on** 

Para iniciar la reproducci´on de los datos grabados, utilicen el comando nativo de ROS 2: 

7 

```
ros2 bag play nombre_de_la_carpeta_del_bag
```

Al ejecutarse, los t´opicos comenzar´an a publicar de forma transparente para sus nodos de procesamiento. 

## **3.3. Odometr´ıa** 

La odometr´ıa a utilizar se publica bajo el t´opico `tb4_0/odom` . Al provenir de un entorno real, posee un nivel de ruido y deriva acumulada caracter´ıstico de los sensores f´ısicos. Apliquen las diferencias temporales discretas para extraer las transformaciones relativas ( _δθ_ 1 _, δθ_ 2 _, δtrans_ ). 

## **3.4. SLAM en ROS 2** 

El objetivo es estructurar el algoritmo de Graph SLAM para estimar la trayectoria bas´andose en las restricciones relativas inducidas por la odometr´ıa y las visualizaciones de los ArUco Tags. Noten que en este caso pr´actico no se dispone de _ground truth_ (posici´on real). Es crucial estructurar un buen criterio de cierre de lazo para corregir la deriva acumulada al regresar a puntos previamente visitados. 

## **3.5. Visualizaci´on con RViz** 

Para validar el comportamiento del estimador basado en grafos, configuren RViz visualizando los siguientes elementos: 

- `tb4_0/scan` : Nube de puntos proveniente del LIDAR real. 

- `tb4_0/odom` : Trayectoria estimada originalmente por odometr´ıa. 

- `/belief` : Trayectoria corregida tras la optimizaci´on del grafo. 

- `/landmarks` : Posiciones estimadas de los marcadores ArUco en el mapa. 

- `/poses_guardadas` : Visualizaci´on de los nodos que componen el grafo de optimizaci´on. 

- `/map` : Grilla de ocupaci´on resultante generada a partir de la trayectoria corregida. 

Se sugiere adem´as abrir una ventana de visualizaci´on de im´agenes con las detecciones de los tags dibujadas sobre el video para certificar la estabilidad del detector visual. 

## **3.6. Mapa final** 

El entregable indispensable constar´a del mapa en formato de grilla de ocupaci´on (m´etrica) y el archivo con las localizaciones relativas de los hitos visuales identificados por sus respectivos IDs. 

8 

## **4. Evaluaci´on del mapa** 

Para determinar la calidad, precisi´on t´ecnica y la correcta implementaci´on del algoritmo de SLAM seleccionado, el cuerpo docente evaluar´a rigurosamente los siguientes t´opicos: 

## **4.1. Coherencia con el entorno real o simulado** 

El mapa final debe reflejar de manera fidedigna la geometr´ıa del entorno f´ısico o virtual propuesto. Las paredes, esquinas y pasillos deben estar definidos n´ıtidamente. No se admitir´an distorsiones geom´etricas severas, solapamientos extra˜nos de muros ni la presencia de obst´aculos fantasma (ruido artificial) o la omisi´on de paredes reales. El entorno debe ser explorado y mapeado en su totalidad. 

## **4.2. Resoluci´on y nivel de detalle** 

La resoluci´on espacial elegida para la grilla o la precisi´on en la convergencia de los hitos geom´etricos debe ser suficiente para discernir detalles cr´ıticos (aperturas de puertas, giros cerrados, pasajes estrechos). El mapa debe estar libre de ruido disperso y no presentar discontinuidades o saltos abruptos que inhabiliten su posterior uso en algoritmos de planificaci´on. 

## **4.3. Consistencia temporal** 

El mapa debe permanecer estable a medida que transcurre el tiempo y el robot vuelve a pasar por zonas conocidas; las actualizaciones consecutivas no deben provocar fluctuaciones err´aticas ni divergencias en las estructuras ya consolidadas. El estimador debe mantener un rastreo coherente de la pose sin perderse o romper la topolog´ıa del entorno durante la exploraci´on. 

## **4.4. Uso para navegaci´on** 

El criterio definitivo de aceptaci´on del mapa es su viabilidad operativa. La grilla generada debe ser apta para que un planificador de caminos global (como A* o Dijkstra) pueda trazar rutas seguras entre coordenadas arbitrarias del laberinto. Se evaluar´a la capacidad del robot para localizarse con precisi´on matem´atica y trasladarse con ´exito sobre el mapa dise˜nado por los alumnos. 

## **5. Entregables** 

Se solicita adjuntar los paquetes de ROS creados, incluyendo archivos de lanzamiento ( _launch files_ ) configurados y documentaci´on clara para su ejecuci´on en la defensa del trabajo. 

9 

