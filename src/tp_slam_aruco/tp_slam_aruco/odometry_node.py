#!/usr/bin/env python3
"""
odometry_node.py
================
Nodo ROS2 que consume la odometría cruda del TurtleBot4 (tópico tb4_0/odom)
y calcula los deltas de movimiento (δrot1, δtrans, δrot2) entre lecturas
consecutivas, usando la matemática de motion_model.py.

POR QUÉ ESTE NODO EXISTE:
El graph_slam_node necesita "restricciones de movimiento" (edges) entre
nodos de pose consecutivos. En vez de pasarle la pose absoluta (que deriva),
le pasamos el incremento relativo entre dos instantes. Este nodo se encarga
de esa conversión y nada más: una sola responsabilidad.

ENTRADA  : nav_msgs/Odometry en 'tb4_0/odom'
SALIDA   : (por ahora) lo logueamos. En la integración final publicaremos
           un mensaje custom o lo consumirá directamente el nodo de SLAM.

NOTA sobre el modelo de deltas (enunciado, sección 3.3): la odometría real
tiene ruido y deriva; por eso trabajamos con diferencias temporales
discretas respecto del instante anterior, no con la pose absoluta.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from tp_slam_aruco.motion_model import (
    compute_delta,
    yaw_from_quaternion,
    OdometryDelta,
)


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # --- Parámetros configurables desde launch/yaml ---
        self.declare_parameter('odom_topic', 'tb4_0/odom')
        # Umbral mínimo de movimiento para registrar un nuevo delta.
        # Evita acumular ruido cuando el robot está quieto.
        self.declare_parameter('min_trans', 0.01)   # 1 cm
        self.declare_parameter('min_rot', 0.01)      # ~0.57 grados

        odom_topic = self.get_parameter('odom_topic').value
        self.min_trans = self.get_parameter('min_trans').value
        self.min_rot = self.get_parameter('min_rot').value

        # Estado interno: la última pose que procesamos.
        # None hasta que llegue el primer mensaje.
        self.last_pose = None

        # Suscripción a la odometría cruda.
        self.sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )

        self.get_logger().info(
            f'odometry_node escuchando en "{odom_topic}". '
            f'Umbrales: trans>{self.min_trans} m, rot>{self.min_rot} rad.'
        )

    def odom_callback(self, msg: Odometry):
        """Se ejecuta cada vez que llega un mensaje de odometría."""
        # Extraemos la pose 2D (x, y, yaw) del mensaje.
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        current_pose = (p.x, p.y, yaw)

        # Primer mensaje: no hay con qué comparar todavía.
        if self.last_pose is None:
            self.last_pose = current_pose
            self.get_logger().info('Primera pose registrada (origen relativo).')
            return

        # Calculamos el delta respecto de la pose anterior.
        delta = compute_delta(self.last_pose, current_pose)

        # Filtro: ignoramos movimientos por debajo del umbral (ruido).
        if delta.trans < self.min_trans and abs(delta.rot1) < self.min_rot \
                and abs(delta.rot2) < self.min_rot:
            return

        # Por ahora logueamos el delta. En la integración con el grafo,
        # acá es donde se agregará un edge entre el nodo anterior y el nuevo.
        self.get_logger().info(
            f'Δ rot1={delta.rot1:+.3f} rad  '
            f'trans={delta.trans:.3f} m  '
            f'rot2={delta.rot2:+.3f} rad'
        )

        # Actualizamos el estado para el próximo callback.
        self.last_pose = current_pose


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
