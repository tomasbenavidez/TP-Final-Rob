#!/usr/bin/env python3
"""coverage_drive: recorre custom_casa para que sim_mapper construya el mapa.

Herramienta de desarrollo (NO es un nodo de la pila de navegación de Parte B):
sólo mueve el TB3 por el entorno simulado mientras `sim_mapper` acumula el mapa
de ocupación. Estrategia tipo Roomba (rebote con giro aleatorio): avanza recto
hasta detectar un obstáculo al frente, gira en el lugar hacia el lado más
despejado durante un tiempo aleatorio, y sigue. Con el LIDAR de 3.5 m, recorrer
el espacio libre así barre las paredes de todos los ambientes. Es robusto a
quedarse en círculos (a diferencia de un seguidor de pared en espacios abiertos).

USO:
  source docs/parte_b/scripts/setup_parte_b.sh
  python3 docs/parte_b/scripts/coverage_drive.py            # indefinido
  python3 docs/parte_b/scripts/coverage_drive.py --secs 240 # 240 s y para

/scan: angle_min=0 (frente = +x del robot), sentido antihorario.
"""

import argparse
import math
import random
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def _sector_min(ranges, angle_min, angle_inc, center_deg, half_deg, rmax):
    c = math.radians(center_deg)
    h = math.radians(half_deg)
    best = rmax
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0:
            continue
        a = angle_min + i * angle_inc
        d = math.atan2(math.sin(a - c), math.cos(a - c))
        if abs(d) <= h and r < best:
            best = r
    return best


class CoverageDrive(Node):
    def __init__(self):
        super().__init__('coverage_drive')
        self.v = 0.15            # avance [m/s]
        self.w_turn = 0.9        # giro en el lugar [rad/s]
        self.front_stop = 0.55   # umbral de obstáculo frontal [m]
        self.side_stop = 0.30    # umbral lateral muy cercano [m]
        self.rmax = 3.5

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.tick)

        self.front = self.left = self.right = self.rmax
        self.rear = self.rmax
        self.have_scan = False
        self.state = 'fwd'
        self.turn_dir = 1.0
        self.turn_until = 0.0
        self.escape_until = 0.0
        # Detección de atasco: si no progresa >0.12 m en 6 s, escapar.
        self.pos = None
        self.last_progress_pos = None
        self.last_progress_t = time.time()
        random.seed()
        self.get_logger().info('coverage_drive: wanderer (rebote + escape) activo.')

    def scan_cb(self, msg: LaserScan):
        a0, ai = msg.angle_min, msg.angle_increment
        self.front = _sector_min(msg.ranges, a0, ai, 0.0, 25.0, self.rmax)
        self.left = _sector_min(msg.ranges, a0, ai, 60.0, 35.0, self.rmax)
        self.right = _sector_min(msg.ranges, a0, ai, -60.0, 35.0, self.rmax)
        self.rear = _sector_min(msg.ranges, a0, ai, 180.0, 30.0, self.rmax)
        self.have_scan = True

    def odom_cb(self, msg: Odometry):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _start_turn(self):
        # Girar hacia el lado MÁS despejado; duración aleatoria (rebote).
        self.turn_dir = 1.0 if self.left >= self.right else -1.0
        self.turn_until = time.time() + random.uniform(0.7, 1.8)
        self.state = 'turn'

    def _check_stuck(self):
        if self.pos is None:
            return False
        if self.last_progress_pos is None:
            self.last_progress_pos = self.pos
            self.last_progress_t = time.time()
            return False
        d = math.hypot(self.pos[0] - self.last_progress_pos[0],
                       self.pos[1] - self.last_progress_pos[1])
        if d > 0.12:
            self.last_progress_pos = self.pos
            self.last_progress_t = time.time()
            return False
        return (time.time() - self.last_progress_t) > 6.0

    def tick(self):
        if not self.have_scan:
            return
        now = time.time()

        # Escape: retroceder (si hay lugar atrás) + girar fuerte, para zafar de
        # esquinas donde el robot queda encajado girando en el lugar.
        if self.state != 'escape' and self._check_stuck():
            self.state = 'escape'
            self.escape_until = now + 2.2
            self.turn_dir = 1.0 if self.left >= self.right else -1.0
            self.last_progress_t = now  # reset para no re-disparar inmediato

        cmd = Twist()
        if self.state == 'escape':
            if now < self.escape_until:
                cmd.linear.x = -0.10 if self.rear > 0.25 else 0.0
                cmd.angular.z = self.w_turn * self.turn_dir
            else:
                self.state = 'fwd'
                self.last_progress_pos = self.pos
                self.last_progress_t = now
            self.pub.publish(cmd)
            return

        blocked = (self.front < self.front_stop or
                   self.left < self.side_stop or self.right < self.side_stop)
        if self.state == 'fwd':
            if blocked:
                self._start_turn()
            else:
                cmd.linear.x = self.v
        if self.state == 'turn':
            if now >= self.turn_until and self.front > self.front_stop * 1.1:
                self.state = 'fwd'
                cmd.linear.x = self.v
            else:
                cmd.angular.z = self.w_turn * self.turn_dir
        self.pub.publish(cmd)

    def stop(self):
        self.pub.publish(Twist())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--secs', type=float, default=0.0)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = CoverageDrive()
    deadline = time.time() + args.secs if args.secs > 0 else None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if deadline and time.time() >= deadline:
                break
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
