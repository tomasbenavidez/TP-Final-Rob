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

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tp_a_slam_aruco.motion_model import (
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
        self.declare_parameter('min_rot', 0.1)      # ~6 grados

        odom_topic = self.get_parameter('odom_topic').value
        self.min_trans = self.get_parameter('min_trans').value
        self.min_rot = self.get_parameter('min_rot').value

        # Estado interno: la última pose que procesamos.
        # None hasta que llegue el primer mensaje.
        self.ref_pose = None

        # Suscripción a la odometría del robot.
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )
        self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_profile=odom_qos
        )

        self.delta_pub = self.create_publisher(Odometry, '/odom_delta', 10)

        self.get_logger().info(
            f'odometry_node escuchando en "{odom_topic}". '
            f'Umbrales: trans>{self.min_trans} m, rot>{self.min_rot} rad.'
        )

    def odom_callback(self, msg: Odometry):
        """Se ejecuta cada vez que llega un mensaje de odometría."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        current_pose = (p.x, p.y, yaw)

        # Primer mensaje: fijamos la referencia inicial.
        if self.ref_pose is None:
            self.ref_pose = current_pose
            self.get_logger().info('Primera pose registrada (origen relativo).')
            return

        # Delta ACUMULADO desde la última pose de referencia (no desde la
        # muestra inmediata anterior). Así atan2 opera sobre un desplazamiento
        # real, no sobre ruido sub-milimétrico.
        delta = compute_delta(self.ref_pose, current_pose)

        # ¿Nos movimos lo suficiente como para emitir un delta?
        if delta.trans < self.min_trans and abs(delta.rot1) < self.min_rot \
                and abs(delta.rot2) < self.min_rot:
            return   # todavía no: NO tocamos ref_pose, seguimos acumulando

        self.get_logger().info(
            f'Δ rot1={delta.rot1:+.3f} rad  '
            f'trans={delta.trans:.3f} m  '
            f'rot2={delta.rot2:+.3f} rad'
        )

        # Pose relativa (frame local del robot): dx, dy en el eje del robot previo,
        # dtheta = rot1 + rot2.
        dx_local = delta.trans * math.cos(delta.rot1)
        dy_local = delta.trans * math.sin(delta.rot1)
        dtheta = delta.rot1 + delta.rot2

        delta_msg = Odometry()
        delta_msg.header.stamp = msg.header.stamp
        delta_msg.header.frame_id = 'base_link'
        delta_msg.child_frame_id = 'base_link'
        delta_msg.pose.pose.position.x = dx_local
        delta_msg.pose.pose.position.y = dy_local
        half = dtheta * 0.5
        delta_msg.pose.pose.orientation.z = math.sin(half)
        delta_msg.pose.pose.orientation.w = math.cos(half)
        self.delta_pub.publish(delta_msg)

        # Recién ahora movemos la referencia al punto actual.
        self.ref_pose = current_pose


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
