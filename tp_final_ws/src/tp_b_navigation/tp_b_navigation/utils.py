#!/usr/bin/env python3
"""Utilidades geométricas compartidas por los nodos de Parte B."""

import math

from geometry_msgs.msg import Quaternion


def normalize_angle(a):
    """Lleva un ángulo a (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def angle_diff(a, b):
    """Diferencia angular a-b normalizada a (-pi, pi] (robusta a wraparound)."""
    return normalize_angle(a - b)


def yaw_from_quaternion(q):
    """Extrae el yaw (rotación en Z) de un geometry_msgs/Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw):
    """Construye un geometry_msgs/Quaternion a partir de un yaw (planar)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q
