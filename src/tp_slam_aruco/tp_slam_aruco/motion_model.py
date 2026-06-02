"""
motion_model.py
===============
Modelo de movimiento por ODOMETRÍA (Probabilistic Robotics, cap. 5).

Este módulo NO depende de ROS. Es matemática pura, para que puedas
testearlo aislado y entender la mecánica antes de meterlo en un nodo.

La idea central:
La odometría del robot (tópico tb4_0/odom) nos da una pose absoluta
(x, y, theta) en cada instante, PERO acumula deriva (error) con el tiempo.
En vez de confiar en la pose absoluta, descomponemos el movimiento entre
dos instantes consecutivos en 3 "deltas" que son más estables:

    δrot1  : rotación inicial para apuntar hacia donde nos movimos
    δtrans : distancia recta recorrida
    δrot2  : rotación final para llegar a la orientación nueva

Visualmente, el robot va de la pose A a la pose B así:

        B (x', y', θ')
       /
      / δtrans
     /
    A·---- (orientación original θ)
    girá δrot1 para apuntar a B, avanzá δtrans, girá δrot2 para quedar en θ'

Estos deltas son la "restricción de movimiento" (edge) que después
metemos entre dos nodos del grafo de poses en GTSAM.
"""

import math
from dataclasses import dataclass


def normalize_angle(angle: float) -> float:
    """
    Lleva cualquier ángulo al rango (-pi, pi].

    Por qué: si no normalizamos, restar dos ángulos cerca de ±pi da
    saltos de ~2pi que arruinan el cálculo de las rotaciones.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class OdometryDelta:
    """Las 3 componentes del movimiento entre dos poses consecutivas."""
    rot1: float    # δrot1 [rad]
    trans: float   # δtrans [m]
    rot2: float    # δrot2 [rad]


def compute_delta(pose_prev, pose_curr) -> OdometryDelta:
    """
    Calcula (δrot1, δtrans, δrot2) a partir de dos poses absolutas.

    Parámetros
    ----------
    pose_prev : tuple (x, y, theta) -> pose en el instante t-1
    pose_curr : tuple (x, y, theta) -> pose en el instante t

    Retorna
    -------
    OdometryDelta

    Esta es exactamente la inversa del modelo de movimiento: dadas dos
    poses, recupera el movimiento que las conecta.
    """
    x_prev, y_prev, th_prev = pose_prev
    x_curr, y_curr, th_curr = pose_curr

    dx = x_curr - x_prev
    dy = y_curr - y_prev
    trans = math.hypot(dx, dy)  # distancia euclídea = sqrt(dx^2 + dy^2)

    # δrot1: ángulo entre la orientación previa y la dirección de avance.
    # Si el robot casi no se trasladó (trans ~ 0), atan2(dy,dx) es ruido puro,
    # así que en ese caso lo dejamos en 0 para no inventar una rotación.
    if trans < 1e-6:
        rot1 = 0.0
    else:
        rot1 = normalize_angle(math.atan2(dy, dx) - th_prev)

    # δrot2: lo que falta girar para llegar a la orientación final,
    # después de haber hecho δrot1.
    rot2 = normalize_angle(th_curr - th_prev - rot1)

    return OdometryDelta(rot1=rot1, trans=trans, rot2=rot2)


def apply_delta(pose_prev, delta: OdometryDelta):
    """
    Aplica un delta a una pose para obtener la pose siguiente (modelo directo).

    Es la operación inversa de compute_delta. Sirve para:
      - propagar la pose estimada (dead reckoning),
      - inicializar nodos nuevos del grafo antes de optimizar.

    Retorna tuple (x, y, theta).
    """
    x, y, th = pose_prev
    x_new = x + delta.trans * math.cos(th + delta.rot1)
    y_new = y + delta.trans * math.sin(th + delta.rot1)
    th_new = normalize_angle(th + delta.rot1 + delta.rot2)
    return (x_new, y_new, th_new)


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """
    Extrae el yaw (rotación en el plano, eje Z) de un cuaternión.

    La odometría de ROS publica la orientación como cuaternión, pero en
    SLAM 2D solo nos importa el yaw. Esta es la fórmula estándar para
    convertir quaternion -> yaw.
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)
