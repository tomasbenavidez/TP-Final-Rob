#!/usr/bin/env python3
"""goal_tour: manda una secuencia de goal_pose a la pila de Parte B para recorrer
toda la casa (cobertura para sim_mapper), usando la planificación A* real.

Herramienta de desarrollo. Publica en /goal_pose y espera a que el state_machine
llegue (nav_state == GOAL_REACHED) o a un timeout, y pasa al siguiente.

USO (con la pila de Parte B + sim_mapper ya corriendo):
  source docs/parte_b/scripts/setup_parte_b.sh
  python3 docs/parte_b/scripts/goal_tour.py
"""

import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# Tour de la casa. La puerta al SUR está en el corredor ESTE (x≈+2.4, cruza y=-1.5).
# Ruta: salir del cuarto oeste -> centro -> este -> bajar corredor este -> barrer sur.
GOALS = [
    (-2.4, 0.3), (-1.2, 0.6), (0.0, 0.8),      # salir del cuarto oeste al centro
    (2.4, 1.0), (2.4, -0.5), (2.4, -1.4),       # ir al este y boca del corredor sur
    (2.4, -2.4), (1.0, -2.6), (-0.5, -2.6),     # entrar y barrer el sur
    (-2.0, -2.6), (-2.6, -2.0), (0.5, -2.4),    # esquinas del sur
    (2.4, -1.4), (2.4, 1.0), (0.0, 0.8),        # volver al norte
]


class GoalTour(Node):
    def __init__(self):
        super().__init__('goal_tour')
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.state = 'IDLE'
        self.create_subscription(String, '/nav_state',
                                 lambda m: setattr(self, 'state', m.data), 10)

    def send(self, x, y):
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = float(x)
        g.pose.position.y = float(y)
        g.pose.orientation.w = 1.0
        self.pub.publish(g)


def main():
    rclpy.init()
    node = GoalTour()
    # esperar a estar localizado
    t0 = time.time()
    while rclpy.ok() and node.state in ('IDLE', 'LOCALIZING') and time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.get_logger().info(f'Arranca tour. estado={node.state}')

    for i, (x, y) in enumerate(GOALS):
        node.send(x, y)
        node.get_logger().info(f'--> goal {i+1}/{len(GOALS)} ({x},{y})')
        t0 = time.time()
        reached = False
        # esperar GOAL_REACHED o timeout 70 s
        while rclpy.ok() and time.time() - t0 < 70:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.state == 'GOAL_REACHED':
                reached = True
                break
        node.get_logger().info(
            f'    goal {i+1} {"OK" if reached else "timeout"} (estado={node.state})')
        # reenviar goal por si el SM volvió a WAITING_GOAL sin que lo viéramos
        time.sleep(0.5)

    node.get_logger().info('Tour de goals completo.')
    node.destroy_node()
    if rclpy.ok():
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
