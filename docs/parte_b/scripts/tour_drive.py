#!/usr/bin/env python3
"""tour_drive: recorrido dirigido de custom_casa para sim_mapper (cobertura completa).

Herramienta de desarrollo (no es nodo de la pila de Parte B). A diferencia del
wanderer aleatorio (coverage_drive.py), recorre una lista de waypoints que barre
TODA la casa (incluido el sur), con go-to-goal proporcional + evasión frontal
reactiva. Cada waypoint tiene timeout para no trabarse. Sirve para que sim_mapper
cubra el entorno completo de forma reproducible.

Pose desde /odom (en Gazebo ≈ ground-truth; map≡odom en el origen).

USO:
  source docs/parte_b/scripts/setup_parte_b.sh
  python3 docs/parte_b/scripts/tour_drive.py
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def sector_min(ranges, a0, ai, center_deg, half_deg, rmax):
    c = math.radians(center_deg); h = math.radians(half_deg); best = rmax
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0:
            continue
        a = a0 + i * ai
        d = math.atan2(math.sin(a - c), math.cos(a - c))
        if abs(d) <= h and r < best:
            best = r
    return best


# Barrido boustrophedon de la casa (~6.7x6.7 m). Reactivo a paredes, así que
# alcanza con un sweep grueso; los waypoints inalcanzables se saltan por timeout.
def build_waypoints():
    wps = []
    rows = [2.6, 1.6, 0.6, -0.4, -1.4, -2.2, -3.0]
    xs = [-2.8, -1.4, 0.0, 1.4, 2.8]
    for k, y in enumerate(rows):
        seq = xs if k % 2 == 0 else list(reversed(xs))
        for x in seq:
            wps.append((x, y))
    return wps


class TourDrive(Node):
    def __init__(self):
        super().__init__('tour_drive')
        self.v_max = 0.16
        self.w_max = 1.2
        self.front_stop = 0.45
        self.reach = 0.35
        self.wp_timeout = 30.0
        self.rmax = 3.5

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.tick)

        self.pose = None
        self.front = self.left = self.right = self.rmax
        self.wps = build_waypoints()
        self.idx = 0
        self.wp_t0 = time.time()
        self.get_logger().info(f'tour_drive: {len(self.wps)} waypoints.')

    def scan_cb(self, m):
        a0, ai = m.angle_min, m.angle_increment
        self.front = sector_min(m.ranges, a0, ai, 0.0, 25.0, self.rmax)
        self.left = sector_min(m.ranges, a0, ai, 55.0, 35.0, self.rmax)
        self.right = sector_min(m.ranges, a0, ai, -55.0, 35.0, self.rmax)

    def odom_cb(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def tick(self):
        if self.pose is None:
            return
        if self.idx >= len(self.wps):
            self.pub.publish(Twist())
            return
        x, y, th = self.pose
        tx, ty = self.wps[self.idx]
        d = math.hypot(tx - x, ty - y)
        if d < self.reach or (time.time() - self.wp_t0) > self.wp_timeout:
            self.idx += 1
            self.wp_t0 = time.time()
            tag = 'OK' if d < self.reach else 'timeout'
            self.get_logger().info(f'wp {self.idx}/{len(self.wps)} ({tag}) pos=({x:.1f},{y:.1f})')
            return
        cmd = Twist()
        if self.front < self.front_stop:
            # Obstáculo: girar en el lugar hacia el lado más despejado.
            cmd.angular.z = self.w_max * (1.0 if self.left >= self.right else -1.0)
        else:
            head = math.atan2(ty - y, tx - x)
            err = math.atan2(math.sin(head - th), math.cos(head - th))
            cmd.angular.z = max(-self.w_max, min(self.w_max, 1.6 * err))
            # avanzar sólo si razonablemente encarado
            cmd.linear.x = self.v_max * max(0.0, math.cos(err)) if abs(err) < 1.0 else 0.0
        self.pub.publish(cmd)

    def done(self):
        return self.idx >= len(self.wps)

    def stop(self):
        self.pub.publish(Twist())


def main():
    rclpy.init()
    node = TourDrive()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info('Tour completo.')
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
