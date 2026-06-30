#!/usr/bin/env python3
"""mcl_localization: Localización Monte Carlo (filtro de partículas) sobre un mapa y un
conjunto de landmarks CONOCIDOS y FIJOS (Sistema 3 de Parte B).

A diferencia de FastSLAM (tps_viejos/tp5), acá los landmarks NO se estiman: vienen dados
por /landmarks (frame map) y la asociación de datos es por índice (el sensor virtual
publica una observación por landmark, en el mismo orden, con (0,0,0) si no es visible).

Flujo:
  - Predicción: por cada /odom se calcula el delta de movimiento (modelo de odometría
    δrot1, δtrans, δrot2) y se propaga cada partícula con ruido (sample motion model).
  - Corrección: por cada /observed_landmarks se pesa cada partícula con la verosimilitud
    gaussiana de las observaciones range/bearing respecto de los landmarks conocidos.
  - Resampling low-variance cuando el número efectivo de partículas cae por debajo de N/2.
  - Inicialización: /initialpose (RViz "2D Pose Estimate") siembra las partículas.
  - Salida: /particlecloud (PoseArray) + /mcl_pose (PoseWithCovarianceStamped) +
    TF map->odom (cierra la cadena map->odom->base_footprint).

Reutiliza la mecánica de pesos/resampling de tps_viejos/tp5/tp5/fastslam_node.py.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)

from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseArray, Pose, PoseWithCovarianceStamped, TransformStamped,
)
from tf2_ros import TransformBroadcaster
from tp_interfaces.msg import LandmarkObservationArray

from tp_b_navigation.utils import (
    normalize_angle, angle_diff, yaw_from_quaternion, quaternion_from_yaw,
)
from tp_b_navigation.landmark_io import load_landmark_map
from tp_b_navigation.safety_gates import measurement_update_due


def predict_particles(
    particles,
    previous_odom,
    current_odom,
    alphas,
    rng,
):
    """Apply the Thrun odometry model and return a propagated particle copy."""
    predicted = np.asarray(particles, dtype=float).copy()
    dx = current_odom[0] - previous_odom[0]
    dy = current_odom[1] - previous_odom[1]
    dtrans = math.hypot(dx, dy)
    drot1 = (
        angle_diff(math.atan2(dy, dx), previous_odom[2])
        if dtrans > 1e-3 else 0.0
    )
    drot2 = angle_diff(
        angle_diff(current_odom[2], previous_odom[2]),
        drot1,
    )
    a1, a2, a3, a4 = alphas
    sd_rot1 = math.sqrt(a1 * drot1 ** 2 + a2 * dtrans ** 2)
    sd_trans = math.sqrt(
        a3 * dtrans ** 2 + a4 * (drot1 ** 2 + drot2 ** 2)
    )
    sd_rot2 = math.sqrt(a1 * drot2 ** 2 + a2 * dtrans ** 2)
    count = len(predicted)
    rot1 = drot1 - rng.normal(0.0, sd_rot1 + 1e-9, count)
    trans = dtrans - rng.normal(0.0, sd_trans + 1e-9, count)
    rot2 = drot2 - rng.normal(0.0, sd_rot2 + 1e-9, count)
    theta = predicted[:, 2].copy()
    predicted[:, 0] += trans * np.cos(theta + rot1)
    predicted[:, 1] += trans * np.sin(theta + rot1)
    predicted[:, 2] = np.arctan2(
        np.sin(theta + rot1 + rot2),
        np.cos(theta + rot1 + rot2),
    )
    return predicted


def map_to_odom_pose(estimate, odom_pose):
    """Return planar T_map_odom from T_map_base and T_odom_base."""
    mx, my, mth = estimate
    ox, oy, oth = odom_pose
    th_mo = normalize_angle(mth - oth)
    cos_t, sin_t = math.cos(th_mo), math.sin(th_mo)
    x_mo = mx - (ox * cos_t - oy * sin_t)
    y_mo = my - (ox * sin_t + oy * cos_t)
    return x_mo, y_mo, th_mo


class MCL(Node):
    def __init__(self):
        super().__init__('mcl_localization')

        # --- Parámetros ---
        self.declare_parameter('num_particles', 350)
        # Ruido del modelo de movimiento de odometría (Thrun, alphas).
        self.declare_parameter('alpha1', 0.05)   # rot por rot
        self.declare_parameter('alpha2', 0.05)   # rot por trans
        self.declare_parameter('alpha3', 0.05)   # trans por trans
        self.declare_parameter('alpha4', 0.05)   # trans por rot
        # Ruido del modelo de medición (para la verosimilitud).
        self.declare_parameter('sigma_range', 0.2)
        self.declare_parameter('sigma_bearing', 0.15)
        # Dispersión inicial al recibir initialpose.
        self.declare_parameter('init_sigma_xy', 0.3)
        self.declare_parameter('init_sigma_yaw', 0.25)
        # Movimiento mínimo (m / rad) para disparar una corrección (ahorro de cómputo).
        self.declare_parameter('update_min_d', 0.02)
        self.declare_parameter('update_min_a', 0.02)
        # Roughening: jitter inyectado tras el resampling para evitar el empobrecimiento
        # (que la nube colapse y pierda el lock al converger muy ajustada).
        self.declare_parameter('rough_xy', 0.03)    # [m]
        self.declare_parameter('rough_yaw', 0.02)   # [rad]
        # Frames.
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('motion_odom_topic', '/odom')
        self.declare_parameter('reference_odom_topic', '/odom')
        self.declare_parameter('tf_publish_rate', 30.0)
        # Tolerancia de transformación: la TF map->odom se publica con el timestamp adelantado
        # este tanto, para que siga siendo válida para lookups a tiempos un poco futuros (p.ej.
        # /observed_landmarks o /scan llegan con un stamp ligeramente más nuevo que la última
        # TF). Es el mismo truco que usa AMCL; sin esto RViz parpadea con "extrapolation into
        # the future". Igual que en AMCL, default 0.1 s.
        self.declare_parameter('transform_tolerance', 0.1)
        self.declare_parameter('landmark_map_file', '')

        self.N = int(self.get_parameter('num_particles').value)
        self.a1 = float(self.get_parameter('alpha1').value)
        self.a2 = float(self.get_parameter('alpha2').value)
        self.a3 = float(self.get_parameter('alpha3').value)
        self.a4 = float(self.get_parameter('alpha4').value)
        self.sigma_r = float(self.get_parameter('sigma_range').value)
        self.sigma_b = float(self.get_parameter('sigma_bearing').value)
        self.init_sxy = float(self.get_parameter('init_sigma_xy').value)
        self.init_syaw = float(self.get_parameter('init_sigma_yaw').value)
        self.update_min_d = float(self.get_parameter('update_min_d').value)
        self.update_min_a = float(self.get_parameter('update_min_a').value)
        self.rough_xy = float(self.get_parameter('rough_xy').value)
        self.rough_yaw = float(self.get_parameter('rough_yaw').value)
        self.global_frame = self.get_parameter('global_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.transform_tolerance = float(self.get_parameter('transform_tolerance').value)

        # --- Estado del filtro ---
        # particles: array Nx3 (x, y, theta) en frame map; weights: array N.
        self.particles = None
        self.weights = None
        self.initialized = False

        self.landmarks = None        # array Mx2 (posiciones conocidas en map)
        self.landmarks_by_id = {}
        self.last_motion_odom = None  # última pose usada por el modelo de movimiento
        self.last_odom_pose = None    # pose de referencia para TF map->odom
        self.accum_d = 0.0           # movimiento acumulado desde la última corrección
        self.accum_a = 0.0
        self.allow_stationary_identified_correction = False
        self.estimate = None         # (x, y, theta) estimado en map

        # --- QoS / pub-sub ---
        qos_latched = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(PoseArray, '/landmarks', self.landmarks_cb, qos_latched)
        motion_odom_topic = str(
            self.get_parameter('motion_odom_topic').value)
        reference_odom_topic = str(
            self.get_parameter('reference_odom_topic').value)
        self.create_subscription(
            Odometry, motion_odom_topic, self.motion_odom_cb, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, reference_odom_topic, self.reference_odom_cb, qos_profile_sensor_data)
        self.create_subscription(PoseArray, '/observed_landmarks',
                                 self.observation_cb, 10)
        self.create_subscription(
            LandmarkObservationArray, '/observed_landmark_ids',
            self.identified_observation_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.initialpose_cb, 10)

        self.pub_particles = self.create_publisher(PoseArray, '/particlecloud', 10)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/mcl_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        rate = float(self.get_parameter('tf_publish_rate').value)
        self.create_timer(1.0 / rate, self.publish_tf)

        landmark_map_file = self.get_parameter('landmark_map_file').value
        if landmark_map_file:
            self.landmarks_by_id = load_landmark_map(landmark_map_file)
            self.get_logger().info(
                f'Cargados {len(self.landmarks_by_id)} landmarks ArUco con ID.')

        self.get_logger().info(
            f'MCL iniciado con {self.N} partículas '
            f'(movimiento={motion_odom_topic}, referencia={reference_odom_topic}). '
            f'Esperando /initialpose '
            f'(usar "2D Pose Estimate" en RViz).')

    # ------------------------------------------------------------------ entradas
    def landmarks_cb(self, msg: PoseArray):
        self.landmarks = np.array(
            [[p.position.x, p.position.y] for p in msg.poses], dtype=float)
        if not self.landmarks_by_id:
            self.landmarks_by_id = {
                index: tuple(point) for index, point in enumerate(self.landmarks)
            }

    def initialpose_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        # Sembrar partículas con una gaussiana alrededor de la pose indicada.
        self.particles = np.empty((self.N, 3))
        self.particles[:, 0] = np.random.normal(x, self.init_sxy, self.N)
        self.particles[:, 1] = np.random.normal(y, self.init_sxy, self.N)
        self.particles[:, 2] = np.random.normal(yaw, self.init_syaw, self.N)
        self.weights = np.full(self.N, 1.0 / self.N)
        self.initialized = True
        self.accum_d = 0.0
        self.accum_a = 0.0
        self.allow_stationary_identified_correction = True
        self._update_estimate()
        self.get_logger().info(
            f'Pose inicial fijada en ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°).')
        self.publish_particles()
        # Publicar /mcl_pose ya al localizar: las correcciones sólo ocurren al moverse, así
        # que sin esto /mcl_pose no sale con el robot quieto y los consumidores (p. ej. el
        # mission_manager de Parte C, que exige /mcl_pose para arrancar) quedan bloqueados.
        self.publish_pose()

    def motion_odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        cur = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

        if not self.initialized:
            self.last_motion_odom = cur
            return
        if self.last_motion_odom is None:
            self.last_motion_odom = cur
            return

        self._predict(self.last_motion_odom, cur)
        self.last_motion_odom = cur
        self._update_estimate()
        self.publish_particles()
        self.publish_pose()

    def reference_odom_cb(self, msg: Odometry):
        """Guarda T_odom_base para construir la TF sin contaminar la predicción."""
        p = msg.pose.pose
        self.last_odom_pose = (
            p.position.x,
            p.position.y,
            yaw_from_quaternion(p.orientation),
        )

    def observation_cb(self, msg: PoseArray):
        if not self.initialized or self.landmarks is None:
            return
        # Sólo corregimos si hubo movimiento apreciable (evita degeneración estática).
        if not measurement_update_due(
            self.accum_d,
            self.accum_a,
            self.update_min_d,
            self.update_min_a,
        ):
            return
        used = self._correct(msg)
        if used:
            self.accum_d = 0.0
            self.accum_a = 0.0

    def identified_observation_cb(self, msg: LandmarkObservationArray):
        if not self.initialized or not self.landmarks_by_id:
            return
        if not measurement_update_due(
            self.accum_d,
            self.accum_a,
            self.update_min_d,
            self.update_min_a,
            allow_stationary=self.allow_stationary_identified_correction,
        ):
            return
        measurements = [
            (int(obs.landmark_id), float(obs.range_m), float(obs.bearing_rad))
            for obs in msg.observations
            if obs.range_m > 0.0 and int(obs.landmark_id) in self.landmarks_by_id
        ]
        used = self._correct_measurements(measurements)
        if used:
            self.accum_d = 0.0
            self.accum_a = 0.0
            self.allow_stationary_identified_correction = False

    # ------------------------------------------------------------ modelo de movimiento
    def _predict(self, prev, cur):
        """Sample motion model (odometría): propaga cada partícula con ruido."""
        dx = cur[0] - prev[0]
        dy = cur[1] - prev[1]
        dtrans = math.hypot(dx, dy)
        self.accum_d += dtrans
        self.accum_a += abs(angle_diff(cur[2], prev[2]))
        self.particles = predict_particles(
            self.particles,
            previous_odom=prev,
            current_odom=cur,
            alphas=(self.a1, self.a2, self.a3, self.a4),
            rng=np.random,
        )

    # ------------------------------------------------------------ modelo de medición
    def _correct(self, msg: PoseArray):
        """Pesa las partículas con la verosimilitud de las observaciones range/bearing."""
        m = min(len(msg.poses), len(self.landmarks))
        measurements = []
        for index in range(m):
            range_m = msg.poses[index].position.x
            bearing = msg.poses[index].position.z
            if range_m == 0.0 and bearing == 0.0:
                continue
            measurements.append((index, range_m, bearing))
        return self._correct_measurements(measurements)

    def _correct_measurements(self, measurements):
        log_w = np.zeros(self.N)
        used = 0

        px = self.particles[:, 0]
        py = self.particles[:, 1]
        pth = self.particles[:, 2]

        inv_2sr2 = 1.0 / (2.0 * self.sigma_r ** 2)
        inv_2sb2 = 1.0 / (2.0 * self.sigma_b ** 2)

        for landmark_id, r, phi in measurements:
            if landmark_id not in self.landmarks_by_id:
                continue
            used += 1

            lmx, lmy = self.landmarks_by_id[landmark_id]
            dx = lmx - px
            dy = lmy - py
            r_hat = np.hypot(dx, dy)
            phi_hat = np.arctan2(dy, dx) - pth

            dr = r - r_hat
            # diferencia angular vectorizada y normalizada
            dphi = np.arctan2(np.sin(phi - phi_hat), np.cos(phi - phi_hat))

            log_w += -(dr * dr) * inv_2sr2 - (dphi * dphi) * inv_2sb2

        if used == 0:
            return 0

        # De log-pesos a pesos normalizados (estable numéricamente).
        log_w -= np.max(log_w)
        w = np.exp(log_w) * self.weights
        total = w.sum()
        if total <= 0 or not np.isfinite(total):
            self.weights = np.full(self.N, 1.0 / self.N)
        else:
            self.weights = w / total
            n_eff = 1.0 / np.sum(self.weights ** 2)
            if n_eff < self.N / 2.0:
                self._resample()

        self._update_estimate()
        self.publish_particles()
        self.publish_pose()
        return used

    def _resample(self):
        """Resampling low-variance (sistemático)."""
        positions = (np.arange(self.N) + np.random.uniform()) / self.N
        idx = np.zeros(self.N, dtype=int)
        cumsum = np.cumsum(self.weights)
        i = j = 0
        while i < self.N:
            if positions[i] < cumsum[j]:
                idx[i] = j
                i += 1
            else:
                j += 1
        self.particles = self.particles[idx].copy()
        # Roughening: jitter para recuperar diversidad perdida al duplicar partículas.
        self.particles[:, 0] += np.random.normal(0.0, self.rough_xy, self.N)
        self.particles[:, 1] += np.random.normal(0.0, self.rough_xy, self.N)
        self.particles[:, 2] += np.random.normal(0.0, self.rough_yaw, self.N)
        self.weights = np.full(self.N, 1.0 / self.N)

    # ------------------------------------------------------------------ estimación
    def _update_estimate(self):
        """Pose estimada: media pesada (circular para el ángulo)."""
        w = self.weights
        x = float(np.sum(w * self.particles[:, 0]))
        y = float(np.sum(w * self.particles[:, 1]))
        th = self.particles[:, 2]
        s = float(np.sum(w * np.sin(th)))
        c = float(np.sum(w * np.cos(th)))
        yaw = math.atan2(s, c)
        self.estimate = (x, y, yaw)

    # ------------------------------------------------------------------ salidas
    def publish_particles(self):
        if self.particles is None:
            return
        pa = PoseArray()
        pa.header.frame_id = self.global_frame
        pa.header.stamp = self.get_clock().now().to_msg()
        for i in range(self.N):
            pose = Pose()
            pose.position.x = float(self.particles[i, 0])
            pose.position.y = float(self.particles[i, 1])
            pose.orientation = quaternion_from_yaw(float(self.particles[i, 2]))
            pa.poses.append(pose)
        self.pub_particles.publish(pa)

    def publish_pose(self):
        if self.estimate is None:
            return
        x, y, yaw = self.estimate
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = quaternion_from_yaw(yaw)

        # Covarianza empírica de la nube (x, y, yaw) -> matriz 6x6.
        dx = self.particles[:, 0] - x
        dy = self.particles[:, 1] - y
        dth = np.arctan2(np.sin(self.particles[:, 2] - yaw),
                         np.cos(self.particles[:, 2] - yaw))
        w = self.weights
        cov = np.zeros((6, 6))
        cov[0, 0] = float(np.sum(w * dx * dx))
        cov[1, 1] = float(np.sum(w * dy * dy))
        cov[0, 1] = cov[1, 0] = float(np.sum(w * dx * dy))
        cov[5, 5] = float(np.sum(w * dth * dth))
        msg.pose.covariance = cov.flatten().tolist()
        self.pub_pose.publish(msg)

    def publish_tf(self):
        """Publica la TF map->odom derivada de la pose estimada y la odometría actual."""
        if self.estimate is None or self.last_odom_pose is None:
            return
        x_mo, y_mo, th_mo = map_to_odom_pose(
            self.estimate,
            self.last_odom_pose,
        )

        t = TransformStamped()
        # Adelantar el stamp por transform_tolerance: la TF queda válida para lookups a
        # tiempos un poco futuros (scan/observaciones más nuevos) -> sin parpadeo de
        # "extrapolation into the future" en RViz y consumidores.
        future = self.get_clock().now() + Duration(seconds=self.transform_tolerance)
        t.header.stamp = future.to_msg()
        t.header.frame_id = self.global_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = x_mo
        t.transform.translation.y = y_mo
        t.transform.rotation = quaternion_from_yaw(th_mo)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MCL()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
