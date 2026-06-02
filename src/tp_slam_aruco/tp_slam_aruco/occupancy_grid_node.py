#!/usr/bin/env python3
"""
occupancy_grid_node.py
=====================
SEGUNDA PASADA del SLAM (tarea F5). Con la trayectoria YA corregida por el
Graph SLAM, reproyecta las lecturas del LIDAR para construir una grilla de
ocupación métrica y consistente, que se exporta como .pgm + .yaml.

================================================================
  ESTE ARCHIVO ES UN ESQUELETO (tarea F5 en el tablero).
  Depende de que el Graph SLAM (F4) ya entregue la trayectoria
  corregida. Por eso va después en el roadmap (Semana 2).
================================================================

POR QUÉ DOS PASADAS (enunciado 3):
  1ª pasada: el grafo resuelve DÓNDE estuvo el robot (trayectoria) usando
             odometría + ArUco. Acá no nos preocupamos por el mapa de paredes.
  2ª pasada: con la trayectoria ya fija y corregida, "pintamos" cada barrido
             del LIDAR en la grilla. Como la trayectoria ya no deriva, las
             paredes no salen dobladas ni duplicadas.

ALGORITMO (inverse sensor model + log-odds):
  Para cada rayo del LIDAR desde una pose conocida:
    - las celdas que el rayo atraviesa hasta el impacto -> LIBRES (resto log-odds)
    - la celda del punto de impacto -> OCUPADA (suma log-odds)
  Acumular en log-odds evita que se sature y permite que múltiples pasadas
  refuercen la evidencia (clave para la consistencia temporal, criterio 4.3).

ENTRADA  : trayectoria corregida (del graph_slam_node) + tb4_0/scan
SALIDA   : nav_msgs/OccupancyGrid en /map + exportación a archivos .pgm/.yaml
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')

        # --- Parámetros de la grilla ---
        self.declare_parameter('resolution', 0.05)   # metros por celda (5 cm)
        self.declare_parameter('width', 600)          # celdas
        self.declare_parameter('height', 600)         # celdas
        self.declare_parameter('scan_topic', 'tb4_0/scan')

        self.resolution = self.get_parameter('resolution').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # La grilla en log-odds (a inicializar en F5).
        # self.log_odds = np.zeros((self.height, self.width))

        self.get_logger().info(
            'occupancy_grid_node iniciado (ESQUELETO). '
            f'Grilla {self.width}x{self.height} @ {self.resolution} m/celda. '
            'La proyección del LIDAR se implementa en F5.'
        )

    def integrate_scan(self, robot_pose, scan: LaserScan):
        """Proyecta un barrido del LIDAR a la grilla desde una pose corregida."""
        raise NotImplementedError('F5: inverse sensor model + actualización log-odds')

    def export_map(self, path_prefix: str):
        """Exporta la grilla a <prefix>.pgm + <prefix>.yaml (formato map_server)."""
        raise NotImplementedError('F5: escribir .pgm (imagen) y .yaml (metadata)')


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
