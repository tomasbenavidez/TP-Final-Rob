#!/usr/bin/env python3
"""
graph_slam_node.py
==================
EL CORAZÓN de la Opción 3. Construye un grafo de poses + landmarks con GTSAM,
detecta cierres de lazo (loop closure) y optimiza para corregir la deriva.

================================================================
  ESTE ARCHIVO ES UN ESQUELETO (tarea F4 en el tablero).
  La lógica de optimización se completa en la sesión dedicada.
  Acá dejamos definido el CONTRATO y la estructura para entender
  cómo encajan las piezas.
================================================================

QUÉ ES UN GRAFO DE POSES (intuición):
  - NODOS: poses del robot a lo largo del tiempo (x_0, x_1, x_2, ...)
           y posiciones de los landmarks ArUco (l_0, l_1, ...).
  - EDGES (restricciones / factores):
      * de movimiento: conectan x_t con x_{t+1} usando el delta de odometría
        (viene del odometry_node). Dicen "me moví así entre estas dos poses".
      * de observación: conectan una pose x_t con un landmark l_k cuando el
        robot ve el ArUco k (viene del aruco_detector_node). Dicen "desde
        acá vi el landmark a este rango y ángulo".
  - LOOP CLOSURE: cuando volvemos a observar un landmark ya visto, el factor
    de observación "tira" del grafo para cerrar el lazo y corregir la deriva.

GTSAM optimiza todo el grafo a la vez (least-squares no lineal) buscando la
configuración de poses+landmarks que mejor explica TODAS las restricciones.

POR QUÉ GTSAM Y NO HACERLO A MANO:
  GTSAM maneja la optimización no lineal (Levenberg-Marquardt / iSAM2),
  el álgebra de manifolds SE(2) y la propagación de covarianzas. Escribir
  eso a mano es la mitad de un TP en sí mismo.

ENTRADA  : deltas de odometría + observaciones de ArUco
SALIDA   : /belief (pose corregida), /poses_guardadas (nodos del grafo),
           /landmarks (posiciones optimizadas). El insumo de occupancy_grid.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

# import gtsam   # <- se activa cuando implementemos la optimización (F4)


class GraphSlamNode(Node):
    def __init__(self):
        super().__init__('graph_slam_node')

        self.declare_parameter('odom_topic', 'tb4_0/odom')

        # --- Estructuras del grafo (a poblar en F4) ---
        # self.graph = gtsam.NonlinearFactorGraph()
        # self.initial_estimate = gtsam.Values()
        self.pose_count = 0           # cuántas poses (nodos) llevamos
        self.seen_landmarks = {}      # id_aruco -> índice de landmark en el grafo

        # Modelos de ruido (covarianzas). Se calibran contra el bag:
        #   - ruido de odometría: F3-B (alpha1..4)
        #   - ruido de observación ArUco: F1-B (covarianza por distancia)
        # self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas([sx, sy, sth])
        # self.obs_noise  = gtsam.noiseModel.Diagonal.Sigmas([srange, sbearing])

        self.get_logger().info(
            'graph_slam_node iniciado (ESQUELETO). '
            'La construcción y optimización del grafo se implementa en F4.'
        )

    # --- Pasos del algoritmo, que implementaremos en la sesión de F4 ---

    def add_odometry_factor(self, delta):
        """Agrega un edge de movimiento entre la pose actual y la siguiente."""
        raise NotImplementedError('F4: agregar factor BetweenFactorPose2')

    def add_observation_factor(self, pose_index, landmark_id, range_, bearing):
        """
        Agrega un edge de observación pose->landmark.
        Si el landmark_id ya estaba visto, este factor genera el LOOP CLOSURE.
        """
        raise NotImplementedError('F4: agregar BearingRangeFactor2D')

    def optimize(self):
        """Corre la optimización no lineal sobre todo el grafo (LM / iSAM2)."""
        raise NotImplementedError('F4: LevenbergMarquardtOptimizer o iSAM2')

    def publish_belief(self):
        """Publica la pose corregida y los landmarks optimizados para RViz."""
        raise NotImplementedError('F4: publicar /belief, /landmarks, /poses_guardadas')


def main(args=None):
    rclpy.init(args=args)
    node = GraphSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
