#!/usr/bin/env python3
"""mcl_localization: Localización Monte Carlo híbrida sobre mapa estático.

A diferencia de FastSLAM (tps_viejos/tp5), acá los landmarks NO se estiman: vienen dados
por /landmarks (frame map) y la asociación de datos es por índice (el sensor virtual
publica una observación por landmark, en el mismo orden, con (0,0,0) si no es visible).

El mapa NO se estima acá: viene de Parte A o del mapa simulado instalado. El
filtro sólo estima la pose del robot.

Flujo:
  - Predicción: por cada /odom se calcula el delta de movimiento (modelo de odometría
    δrot1, δtrans, δrot2) y se propaga cada partícula con ruido (sample motion model).
  - Corrección: por cada /observed_landmarks se pesa cada partícula con la verosimilitud
    gaussiana de las observaciones range/bearing respecto de los landmarks conocidos.
  - Corrección láser: por cada /scan se compara una muestra de endpoints contra un
    likelihood field derivado de /map.
  - Resampling low-variance cuando el número efectivo de partículas cae por debajo de N/2.
  - Inicialización: /initialpose (RViz "2D Pose Estimate") siembra las partículas.
  - Salida: /particlecloud (PoseArray) + /mcl_pose (PoseWithCovarianceStamped) +
    TF map->odom (cierra la cadena map->odom->base_footprint).

Reutiliza la mecánica de pesos/resampling de tps_viejos/tp5/tp5/fastslam_node.py.
"""

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

import rclpy
import rclpy.time
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
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (
    PoseArray, Pose, PoseWithCovarianceStamped, TransformStamped,
)
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tp_interfaces.msg import LandmarkObservationArray

from tp_b_navigation.utils import (
    normalize_angle, angle_diff, yaw_from_quaternion, quaternion_from_yaw,
)
from tp_b_navigation.landmark_io import load_landmark_map
from tp_b_navigation.mcl_diagnostics import MclDiagnosticsCsv
from tp_b_navigation.mcl_models import (
    LikelihoodField,
    LandmarkMeasurement,
    landmark_log_likelihood,
    laser_scan_log_likelihood,
    legacy_pose_observations_to_measurements,
)
from tp_b_navigation.safety_gates import measurement_update_due


@dataclass
class OosSnapshot:
    stamp: float
    odom_pose: tuple[float, float, float]
    particles: np.ndarray
    weights: np.ndarray
    estimate: tuple[float, float, float] | None


@dataclass
class OosMotionStep:
    start_stamp: float
    end_stamp: float
    previous_odom: tuple[float, float, float]
    current_odom: tuple[float, float, float]


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


def _effective_particle_count_for(weights):
    denom = float(np.sum(np.asarray(weights, dtype=float) ** 2))
    if denom <= 0.0 or not math.isfinite(denom):
        return None
    return 1.0 / denom


def _resample_arrays(particles, weights, rough_xy, rough_yaw):
    count = len(particles)
    positions = (np.arange(count) + np.random.uniform()) / count
    idx = np.zeros(count, dtype=int)
    cumsum = np.cumsum(weights)
    i = j = 0
    while i < count:
        if positions[i] < cumsum[j]:
            idx[i] = j
            i += 1
        else:
            j += 1
    resampled = particles[idx].copy()
    resampled[:, 0] += np.random.normal(0.0, rough_xy, count)
    resampled[:, 1] += np.random.normal(0.0, rough_xy, count)
    resampled[:, 2] += np.random.normal(0.0, rough_yaw, count)
    return resampled, np.full(count, 1.0 / count)


def _apply_log_likelihood_to_arrays(
    particles,
    weights,
    log_w,
    rough_xy,
    rough_yaw,
):
    log_w = np.asarray(log_w, dtype=float)
    finite_log_w = log_w[np.isfinite(log_w)]
    if not finite_log_w.size:
        return None
    log_min = float(np.min(finite_log_w))
    log_max = float(np.max(finite_log_w))
    n_eff_before = _effective_particle_count_for(weights)
    log_w = log_w.copy()
    log_w -= np.max(log_w)
    updated = np.exp(log_w) * weights
    total = updated.sum()
    reset_weights = False
    resampled = False
    if total <= 0 or not np.isfinite(total):
        updated = np.full(len(weights), 1.0 / len(weights))
        reset_weights = True
    else:
        updated = updated / total
        n_eff = _effective_particle_count_for(updated)
        if n_eff is not None and n_eff < len(weights) / 2.0:
            particles, updated = _resample_arrays(
                particles,
                updated,
                rough_xy=rough_xy,
                rough_yaw=rough_yaw,
            )
            resampled = True
    return {
        'particles': particles,
        'weights': updated,
        'n_eff_before': n_eff_before,
        'n_eff_after': _effective_particle_count_for(updated),
        'resampled': resampled,
        'reset_weights': reset_weights,
        'log_min': log_min,
        'log_max': log_max,
    }


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
        self.declare_parameter('use_landmark_likelihood', True)
        self.declare_parameter('use_laser_likelihood', True)
        self.declare_parameter('laser_max_beams', 60)
        self.declare_parameter('laser_sigma_hit', 0.18)
        self.declare_parameter('laser_max_distance', 0.8)
        self.declare_parameter('laser_log_weight', 1.0)
        self.declare_parameter('landmark_log_weight', 1.0)
        self.declare_parameter('occupied_pose_penalty', 20.0)
        self.declare_parameter('max_landmark_measurement_age', 0.5)
        self.declare_parameter('use_oos_landmark_updates', False)
        self.declare_parameter('oos_max_observation_age', 4.0)
        self.declare_parameter('oos_history_duration', 6.0)
        self.declare_parameter('oos_max_snapshot_gap', 0.15)
        self.declare_parameter('oos_replay_deterministic', True)
        self.declare_parameter('diagnostics_csv', '')

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
        self.landmark_map_file = str(self.get_parameter('landmark_map_file').value)
        self.use_landmark_likelihood = bool(
            self.get_parameter('use_landmark_likelihood').value)
        self.use_laser_likelihood = bool(
            self.get_parameter('use_laser_likelihood').value)
        self.laser_max_beams = int(self.get_parameter('laser_max_beams').value)
        self.laser_sigma_hit = float(self.get_parameter('laser_sigma_hit').value)
        self.laser_max_distance = float(
            self.get_parameter('laser_max_distance').value)
        self.laser_log_weight = float(self.get_parameter('laser_log_weight').value)
        self.landmark_log_weight = float(
            self.get_parameter('landmark_log_weight').value)
        self.occupied_pose_penalty = float(
            self.get_parameter('occupied_pose_penalty').value)
        self.max_landmark_measurement_age = float(
            self.get_parameter('max_landmark_measurement_age').value)
        self.use_oos_landmark_updates = bool(
            self.get_parameter('use_oos_landmark_updates').value)
        self.oos_max_observation_age = float(
            self.get_parameter('oos_max_observation_age').value)
        self.oos_history_duration = float(
            self.get_parameter('oos_history_duration').value)
        self.oos_max_snapshot_gap = float(
            self.get_parameter('oos_max_snapshot_gap').value)
        self.oos_replay_deterministic = bool(
            self.get_parameter('oos_replay_deterministic').value)
        diagnostics_csv = str(self.get_parameter('diagnostics_csv').value)

        # --- Estado del filtro ---
        # particles: array Nx3 (x, y, theta) en frame map; weights: array N.
        self.particles = None
        self.weights = None
        self.initialized = False

        self.landmarks = None        # array Mx2 (posiciones conocidas en map)
        self.landmarks_by_id = {}
        self.last_motion_odom = None  # última pose usada por el modelo de movimiento
        self.last_motion_stamp = None
        self.last_odom_pose = None    # pose de referencia para TF map->odom
        self.accum_d = 0.0           # movimiento acumulado desde la última corrección
        self.accum_a = 0.0
        self.laser_accum_d = 0.0
        self.laser_accum_a = 0.0
        self.allow_stationary_identified_correction = False
        self.allow_stationary_laser_correction = False
        self.estimate = None         # (x, y, theta) estimado en map
        self.likelihood_field = None
        self.received_identified_landmarks = False
        self.diagnostics = (
            MclDiagnosticsCsv(diagnostics_csv) if diagnostics_csv else None
        )
        self._oos_snapshots = deque()
        self._oos_motion_steps = deque()

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
                                 self.observation_cb, 1)
        self.create_subscription(
            LandmarkObservationArray, '/observed_landmark_ids',
            self.identified_observation_cb, 1)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_latched)
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.initialpose_cb, 10)

        self.pub_particles = self.create_publisher(PoseArray, '/particlecloud', 10)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/mcl_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        rate = float(self.get_parameter('tf_publish_rate').value)
        self.create_timer(1.0 / rate, self.publish_tf)

        if self.landmark_map_file:
            self.landmarks_by_id = load_landmark_map(self.landmark_map_file)
            self.get_logger().info(
                f'Cargados {len(self.landmarks_by_id)} landmarks ArUco con ID.')
        if self.diagnostics is not None:
            self.get_logger().info(
                f'Diagnostico MCL CSV activo: {self.diagnostics.path}')

        self.get_logger().info(
            f'MCL iniciado con {self.N} partículas '
            f'(movimiento={motion_odom_topic}, referencia={reference_odom_topic}). '
            f'Esperando /initialpose '
            f'(usar "2D Pose Estimate" en RViz).')

    # ------------------------------------------------------------------ entradas
    def landmarks_cb(self, msg: PoseArray):
        self.landmarks = np.array(
            [[p.position.x, p.position.y] for p in msg.poses], dtype=float)
        # Si se cargó el JSON de Parte A, los IDs ArUco reales tienen prioridad.
        # No los reemplazamos por índices virtuales de /landmarks.
        if not getattr(self, 'landmark_map_file', '') and not self.landmarks_by_id:
            self.landmarks_by_id = {
                index: tuple(point) for index, point in enumerate(self.landmarks)
            }

    def map_cb(self, msg: OccupancyGrid):
        if not self.use_laser_likelihood:
            return
        self.likelihood_field = LikelihoodField.from_occupancy(
            msg.data,
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            max_distance=self.laser_max_distance,
        )

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
        self.laser_accum_d = 0.0
        self.laser_accum_a = 0.0
        self.allow_stationary_identified_correction = True
        self.allow_stationary_laser_correction = True
        self._clear_oos_history()
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
        stamp = self._message_stamp_sec(msg)
        if stamp is None:
            stamp = getattr(self, 'last_motion_stamp', None)
            if stamp is None:
                try:
                    stamp = self._now_sec()
                except AttributeError:
                    stamp = 0.0

        if not self.initialized:
            self.last_motion_odom = cur
            self.last_motion_stamp = stamp
            return
        if self.last_motion_odom is None:
            self.last_motion_odom = cur
            self.last_motion_stamp = stamp
            self._record_oos_snapshot(stamp, cur)
            return

        prev = self.last_motion_odom
        prev_stamp = getattr(self, 'last_motion_stamp', None)
        self._predict(self.last_motion_odom, cur)
        if prev_stamp is not None:
            self._record_oos_motion_step(prev_stamp, stamp, prev, cur)
        self.last_motion_odom = cur
        self.last_motion_stamp = stamp
        self._update_estimate()
        self._record_oos_snapshot(stamp, cur)
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
        if not getattr(self, 'use_landmark_likelihood', True):
            return
        if getattr(self, 'received_identified_landmarks', False):
            return
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
        measurement_stamp = self._message_stamp_sec(msg)
        if not self._measurement_is_fresh(measurement_stamp):
            return
        used = self._correct(msg)
        if used:
            self.accum_d = 0.0
            self.accum_a = 0.0

    def identified_observation_cb(self, msg: LandmarkObservationArray):
        if not getattr(self, 'use_landmark_likelihood', True):
            return
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
        measurement_stamp = self._message_stamp_sec(msg)
        measurements = [
            LandmarkMeasurement(
                int(obs.landmark_id), float(obs.range_m),
                float(obs.bearing_rad), 'identified')
            for obs in msg.observations
            if obs.range_m > 0.0 and int(obs.landmark_id) in self.landmarks_by_id
        ]
        if measurements:
            self.received_identified_landmarks = True
        if self._measurement_is_fresh(measurement_stamp):
            used = self._correct_measurements(
                measurements,
                source='identified',
                measurement_stamp=measurement_stamp,
            )
        elif self._measurement_can_use_oos(measurement_stamp):
            used = self._correct_measurements_oos(
                measurements,
                source='identified',
                measurement_stamp=measurement_stamp,
            )
        else:
            used = 0
        if used:
            self.accum_d = 0.0
            self.accum_a = 0.0
            self.allow_stationary_identified_correction = False

    def scan_cb(self, msg: LaserScan):
        if not self.use_laser_likelihood:
            return
        if not self.initialized or self.likelihood_field is None:
            return
        if not measurement_update_due(
            self.laser_accum_d,
            self.laser_accum_a,
            self.update_min_d,
            self.update_min_a,
            allow_stationary=self.allow_stationary_laser_correction,
        ):
            return
        try:
            sensor_pose = self._sensor_pose_in_base(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'TF {msg.header.frame_id}->{self.base_frame} no disponible '
                f'para likelihood laser: {exc}',
                throttle_duration_sec=2.0,
            )
            return
        log_w, used = laser_scan_log_likelihood(
            self.particles,
            ranges=msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
            sensor_pose=sensor_pose,
            field=self.likelihood_field,
            max_beams=self.laser_max_beams,
            sigma_hit=self.laser_sigma_hit,
            occupied_pose_penalty=self.occupied_pose_penalty,
            log_weight=self.laser_log_weight,
        )
        detail = f'frame={msg.header.frame_id};beams={used}'
        if self._apply_log_likelihood(
            log_w, used, 'laser', detail,
            measurement_stamp=self._stamp_to_sec(msg.header.stamp),
        ):
            self.laser_accum_d = 0.0
            self.laser_accum_a = 0.0
            self.allow_stationary_laser_correction = False

    # ------------------------------------------------------------ modelo de movimiento
    def _predict(self, prev, cur):
        """Sample motion model (odometría): propaga cada partícula con ruido."""
        dx = cur[0] - prev[0]
        dy = cur[1] - prev[1]
        dtrans = math.hypot(dx, dy)
        self.accum_d += dtrans
        self.accum_a += abs(angle_diff(cur[2], prev[2]))
        self.laser_accum_d = getattr(self, 'laser_accum_d', 0.0) + dtrans
        self.laser_accum_a = (
            getattr(self, 'laser_accum_a', 0.0) +
            abs(angle_diff(cur[2], prev[2]))
        )
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
        measurements = legacy_pose_observations_to_measurements(
            msg.poses[:len(self.landmarks)])
        return self._correct_measurements(
            measurements,
            source='legacy_landmark',
            measurement_stamp=self._message_stamp_sec(msg),
        )

    def _correct_measurements(
        self,
        measurements,
        source='landmark',
        measurement_stamp=None,
    ):
        log_w, used = landmark_log_likelihood(
            self.particles,
            measurements,
            self.landmarks_by_id,
            sigma_range=self.sigma_r,
            sigma_bearing=self.sigma_b,
            log_weight=getattr(self, 'landmark_log_weight', 1.0),
        )
        ids = ';'.join(str(self._measurement_id(m)) for m in measurements)
        detail = f'source={source};ids={ids}'
        return used if self._apply_log_likelihood(
            log_w, used, 'landmark', detail,
            measurement_stamp=measurement_stamp,
        ) else 0

    def _correct_measurements_oos(
        self,
        measurements,
        source='landmark',
        measurement_stamp=None,
    ):
        if not measurements or measurement_stamp is None:
            return 0
        snapshot = self._find_oos_snapshot(measurement_stamp)
        ids = ';'.join(str(self._measurement_id(m)) for m in measurements)
        age = self._now_sec() - measurement_stamp
        if snapshot is None:
            self._record_oos_drop(
                measurement_stamp=measurement_stamp,
                reason='no_history',
                detail=f'source={source};ids={ids};age_sec={age:.3f}',
            )
            return 0
        snapshot_dt = measurement_stamp - snapshot.stamp
        if snapshot_dt < -1e-9 or snapshot_dt > getattr(
            self, 'oos_max_snapshot_gap', 0.15
        ):
            self._record_oos_drop(
                measurement_stamp=measurement_stamp,
                reason='snapshot_gap',
                detail=(
                    f'source={source};ids={ids};age_sec={age:.3f};'
                    f'snapshot_dt_sec={snapshot_dt:.3f}'
                ),
            )
            return 0

        historical_particles = snapshot.particles.copy()
        historical_weights = snapshot.weights.copy()
        log_w, used = landmark_log_likelihood(
            historical_particles,
            measurements,
            self.landmarks_by_id,
            sigma_range=self.sigma_r,
            sigma_bearing=self.sigma_b,
            log_weight=getattr(self, 'landmark_log_weight', 1.0),
        )
        if used == 0:
            return 0
        result = _apply_log_likelihood_to_arrays(
            historical_particles,
            historical_weights,
            log_w,
            rough_xy=getattr(self, 'rough_xy', 0.0),
            rough_yaw=getattr(self, 'rough_yaw', 0.0),
        )
        if result is None:
            return 0

        replayed_particles = result['particles']
        replay_steps = self._oos_motion_steps_after(snapshot.stamp)
        for step in replay_steps:
            replayed_particles = predict_particles(
                replayed_particles,
                previous_odom=step.previous_odom,
                current_odom=step.current_odom,
                alphas=(0.0, 0.0, 0.0, 0.0),
                rng=np.random,
            )

        estimate_before = getattr(self, 'estimate', None)
        self.particles = replayed_particles
        self.weights = result['weights']
        self._update_estimate()
        self._record_diagnostic(
            event='landmark_oos',
            used=used,
            n_eff_before=result['n_eff_before'],
            n_eff_after=result['n_eff_after'],
            resampled=result['resampled'],
            reset_weights=result['reset_weights'],
            estimate_before=estimate_before,
            estimate_after=getattr(self, 'estimate', None),
            measurement_stamp=measurement_stamp,
            log_likelihood_min=result['log_min'],
            log_likelihood_max=result['log_max'],
            detail=(
                f'source={source};ids={ids};age_sec={age:.3f};'
                f'snapshot_dt_sec={snapshot_dt:.3f};'
                f'replay_steps={len(replay_steps)}'
            ),
        )
        last_stamp = getattr(self, 'last_motion_stamp', None)
        last_odom = getattr(self, 'last_motion_odom', None)
        if last_stamp is not None and last_odom is not None:
            self._record_oos_snapshot(last_stamp, last_odom)
        self.publish_particles()
        self.publish_pose()
        return used

    @staticmethod
    def _measurement_id(measurement):
        if hasattr(measurement, 'landmark_id'):
            return measurement.landmark_id
        return measurement[0]

    def _apply_log_likelihood(
        self,
        log_w,
        used,
        event='',
        detail='',
        measurement_stamp=None,
    ):
        if used == 0:
            return False

        log_w = np.asarray(log_w, dtype=float)
        finite_log_w = log_w[np.isfinite(log_w)]
        if finite_log_w.size:
            log_min = float(np.min(finite_log_w))
            log_max = float(np.max(finite_log_w))
        else:
            return False
        estimate_before = getattr(self, 'estimate', None)
        n_eff_before = self._effective_particle_count()
        resampled = False
        reset_weights = False

        # De log-pesos a pesos normalizados (estable numéricamente).
        log_w -= np.max(log_w)
        w = np.exp(log_w) * self.weights
        total = w.sum()
        if total <= 0 or not np.isfinite(total):
            self.weights = np.full(self.N, 1.0 / self.N)
            reset_weights = True
        else:
            self.weights = w / total
            n_eff = 1.0 / np.sum(self.weights ** 2)
            if n_eff < self.N / 2.0:
                self._resample()
                resampled = True

        self._update_estimate()
        self._record_diagnostic(
            event=event,
            used=used,
            n_eff_before=n_eff_before,
            n_eff_after=self._effective_particle_count(),
            resampled=resampled,
            reset_weights=reset_weights,
            estimate_before=estimate_before,
            estimate_after=getattr(self, 'estimate', None),
            measurement_stamp=measurement_stamp,
            log_likelihood_min=log_min,
            log_likelihood_max=log_max,
            detail=detail,
        )
        self.publish_particles()
        self.publish_pose()
        return True

    def _effective_particle_count(self):
        weights = getattr(self, 'weights', None)
        if weights is None:
            return None
        denom = float(np.sum(weights ** 2))
        if denom <= 0.0 or not math.isfinite(denom):
            return None
        return 1.0 / denom

    def _covariance_diag(self):
        if self.estimate is None or self.particles is None or self.weights is None:
            return 0.0, 0.0, 0.0
        x, y, yaw = self.estimate
        dx = self.particles[:, 0] - x
        dy = self.particles[:, 1] - y
        dth = np.arctan2(
            np.sin(self.particles[:, 2] - yaw),
            np.cos(self.particles[:, 2] - yaw),
        )
        return (
            float(np.sum(self.weights * dx * dx)),
            float(np.sum(self.weights * dy * dy)),
            float(np.sum(self.weights * dth * dth)),
        )

    def _record_diagnostic(
        self,
        *,
        event,
        used,
        n_eff_before,
        n_eff_after,
        resampled,
        reset_weights,
        estimate_before,
        estimate_after,
        measurement_stamp,
        log_likelihood_min,
        log_likelihood_max,
        detail,
    ):
        diagnostics = getattr(self, 'diagnostics', None)
        if diagnostics is None:
            return
        stamp = self.get_clock().now()
        stamp_sec = stamp.nanoseconds * 1e-9
        diagnostics.write_update(
            stamp_sec=stamp_sec,
            measurement_stamp_sec=measurement_stamp,
            event=event,
            used=used,
            n_eff_before=n_eff_before,
            n_eff_after=n_eff_after,
            resampled=resampled,
            reset_weights=reset_weights,
            estimate_before=estimate_before,
            estimate_after=estimate_after,
            covariance_diag=self._covariance_diag(),
            weight_max=float(np.max(self.weights)),
            log_likelihood_min=log_likelihood_min,
            log_likelihood_max=log_likelihood_max,
            accum_d=getattr(self, 'accum_d', 0.0),
            accum_a=getattr(self, 'accum_a', 0.0),
            laser_accum_d=getattr(self, 'laser_accum_d', 0.0),
            laser_accum_a=getattr(self, 'laser_accum_a', 0.0),
            detail=detail,
        )

    def _record_oos_drop(self, *, measurement_stamp, reason, detail):
        diagnostics = getattr(self, 'diagnostics', None)
        if diagnostics is None:
            return
        stamp = self.get_clock().now()
        diagnostics.write_update(
            stamp_sec=stamp.nanoseconds * 1e-9,
            measurement_stamp_sec=measurement_stamp,
            event='landmark_oos_drop',
            used=0,
            n_eff_before=self._effective_particle_count(),
            n_eff_after=self._effective_particle_count(),
            resampled=False,
            reset_weights=False,
            estimate_before=getattr(self, 'estimate', None),
            estimate_after=getattr(self, 'estimate', None),
            covariance_diag=self._covariance_diag(),
            weight_max=float(np.max(self.weights)) if self.weights is not None else 0.0,
            log_likelihood_min=0.0,
            log_likelihood_max=0.0,
            accum_d=getattr(self, 'accum_d', 0.0),
            accum_a=getattr(self, 'accum_a', 0.0),
            laser_accum_d=getattr(self, 'laser_accum_d', 0.0),
            laser_accum_a=getattr(self, 'laser_accum_a', 0.0),
            detail=f'reason={reason};{detail}',
        )

    @staticmethod
    def _stamp_to_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _measurement_is_fresh(self, measurement_stamp):
        max_age = getattr(self, 'max_landmark_measurement_age', 0.0)
        if max_age <= 0.0 or measurement_stamp is None:
            return True
        age = self._now_sec() - measurement_stamp
        return age <= max_age

    def _measurement_can_use_oos(self, measurement_stamp):
        if not getattr(self, 'use_oos_landmark_updates', False):
            return False
        if measurement_stamp is None:
            return False
        age = self._now_sec() - measurement_stamp
        max_age = getattr(self, 'oos_max_observation_age', 4.0)
        return 0.0 <= age <= max_age

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _message_stamp_sec(msg):
        header = getattr(msg, 'header', None)
        stamp = getattr(header, 'stamp', None)
        if stamp is None:
            return None
        return MCL._stamp_to_sec(stamp)

    def _clear_oos_history(self):
        self._oos_snapshots = deque()
        self._oos_motion_steps = deque()

    def _record_oos_snapshot(self, stamp, odom_pose):
        if stamp is None or self.particles is None or self.weights is None:
            return
        snapshots = getattr(self, '_oos_snapshots', None)
        if snapshots is None:
            self._oos_snapshots = deque()
            snapshots = self._oos_snapshots
        elif not hasattr(snapshots, 'popleft'):
            self._oos_snapshots = deque(snapshots)
            snapshots = self._oos_snapshots
        snapshots.append(OosSnapshot(
            stamp=float(stamp),
            odom_pose=tuple(odom_pose),
            particles=np.asarray(self.particles, dtype=float).copy(),
            weights=np.asarray(self.weights, dtype=float).copy(),
            estimate=getattr(self, 'estimate', None),
        ))
        self._prune_oos_history(float(stamp))

    def _record_oos_motion_step(self, start_stamp, end_stamp, previous_odom, current_odom):
        if start_stamp is None or end_stamp is None:
            return
        steps = getattr(self, '_oos_motion_steps', None)
        if steps is None:
            self._oos_motion_steps = deque()
            steps = self._oos_motion_steps
        elif not hasattr(steps, 'popleft'):
            self._oos_motion_steps = deque(steps)
            steps = self._oos_motion_steps
        if float(end_stamp) < float(start_stamp):
            return
        steps.append(OosMotionStep(
            start_stamp=float(start_stamp),
            end_stamp=float(end_stamp),
            previous_odom=tuple(previous_odom),
            current_odom=tuple(current_odom),
        ))
        self._prune_oos_history(float(end_stamp))

    def _prune_oos_history(self, newest_stamp):
        cutoff = newest_stamp - getattr(self, 'oos_history_duration', 6.0)
        snapshots = getattr(self, '_oos_snapshots', deque())
        while snapshots and snapshots[0].stamp < cutoff:
            snapshots.popleft()
        steps = getattr(self, '_oos_motion_steps', deque())
        while steps and steps[0].end_stamp < cutoff:
            steps.popleft()

    def _find_oos_snapshot(self, measurement_stamp):
        snapshots = list(getattr(self, '_oos_snapshots', []))
        selected = None
        for snapshot in snapshots:
            if snapshot.stamp <= measurement_stamp:
                selected = snapshot
            else:
                break
        return selected

    def _oos_motion_steps_after(self, snapshot_stamp):
        return [
            step for step in getattr(self, '_oos_motion_steps', [])
            if step.end_stamp > snapshot_stamp
        ]

    def _sensor_pose_in_base(self, scan: LaserScan):
        sensor_frame = scan.header.frame_id or self.base_frame
        if sensor_frame == self.base_frame:
            return 0.0, 0.0, 0.0
        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            sensor_frame,
            rclpy.time.Time.from_msg(scan.header.stamp),
        )
        tr = transform.transform
        return (
            tr.translation.x,
            tr.translation.y,
            yaw_from_quaternion(tr.rotation),
        )

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

    def destroy_node(self):
        diagnostics = getattr(self, 'diagnostics', None)
        if diagnostics is not None:
            diagnostics.close()
        return super().destroy_node()


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
