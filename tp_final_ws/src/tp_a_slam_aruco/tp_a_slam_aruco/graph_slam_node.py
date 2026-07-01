import json
import math
import csv
import os
from bisect import bisect_left
from dataclasses import replace
import numpy as np

import rclpy # type: ignore
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray

import gtsam
from gtsam import Pose2, Point2, Rot2
from gtsam.symbol_shorthand import X, L

from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from tp_a_slam_aruco.motion_model import yaw_from_quaternion, normalize_angle
from tp_a_slam_aruco.slam_gating import (
    innovation_mahalanobis_sq,
    observation_sigmas,
    resolve_gate_state,
    spatial_landmark_gate_from_values,
)
from tp_a_slam_aruco.scan_odom_buffer import interpolate_bracketed
from tp_a_slam_aruco.slam_geometry import (
    CameraExtrinsics,
    TB4_CAMERA_EXTRINSICS,
    fallback_camera_to_base_xy,
    transform_stamped_to_base_xy,
)
from tp_a_slam_aruco.slam_graph import optimize_graph
from tp_a_slam_aruco.slam_io import write_trajectory_json
from tp_a_slam_aruco.slam_debug import ArucoGeometryObservation, build_geometry_debug_row
from tp_a_slam_aruco.slam_landmarks import LandmarkObservationGate
from tp_a_slam_aruco.slam_publish import (
    build_base_debug_markers,
    build_belief_message,
    build_landmark_markers,
    build_map_to_odom_transform,
    build_path_message,
    build_pose_array_message,
)

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.clock import Clock, ClockType


class GraphSlamNode(Node):
    def __init__(self):
        super().__init__('graph_slam_node')

        # ---------------- Parametros ----------------
        self.declare_parameter('odom_topic', 'tb4_0/odom')
        self.declare_parameter('landmarks_topic', '/aruco_detections')
        self.declare_parameter('optimized_landmarks_topic', '/landmarks')
        self.declare_parameter('legacy_landmarks_topic', '/landmarks_opt')
        self.declare_parameter('base_debug_topic', '/aruco_base_debug')
        self.declare_parameter('kf_dist', 0.15)        # m: distancia mínima entre keyframes
        self.declare_parameter('optimize_every', 1)
        self.declare_parameter('kf_angle_max', 0.60)   # rad (~34°): KF por giro grande
        self.declare_parameter('reobs_min_parallax', 0.20)  # m
        self.declare_parameter('maha_threshold', 5.99)    # chi2 2-DOF al 95%
        self.declare_parameter('cauchy_k', 1.0)           # kernel Cauchy (unidades whitened)
        self.declare_parameter('min_marker_depth', 0.15)
        self.declare_parameter('max_marker_depth', 5.0)
        self.declare_parameter('min_landmark_observations', 3)
        self.declare_parameter('max_landmark_position_jump', 0.75)
        self.declare_parameter('detection_keyframe_tolerance', 0.10)
        self.declare_parameter('max_pending_detections', 500)
        self.declare_parameter('max_detection_wait_seconds', 1.0)
        self.declare_parameter('max_odom_buffer_samples', 4000)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # Extrínsecos cámara->base (fallback si no hay TF)
        self.declare_parameter('camera_tx', TB4_CAMERA_EXTRINSICS.tx)
        self.declare_parameter('camera_ty', TB4_CAMERA_EXTRINSICS.ty)
        self.declare_parameter('camera_yaw', TB4_CAMERA_EXTRINSICS.yaw)
        # Ruta JSON para exportar trayectoria+landmarks al finalizar el bag
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('geometry_debug_file', '/tmp/aruco_geometry_debug.csv')

        odom_topic = self.get_parameter('odom_topic').value
        lm_topic = self.get_parameter('landmarks_topic').value
        optimized_landmarks_topic = self.get_parameter('optimized_landmarks_topic').value
        legacy_landmarks_topic = self.get_parameter('legacy_landmarks_topic').value
        base_debug_topic = self.get_parameter('base_debug_topic').value
        self.kf_dist = self.get_parameter('kf_dist').value
        self.optimize_every = self.get_parameter('optimize_every').value
        self.kf_angle_max = self.get_parameter('kf_angle_max').value
        self.reobs_min_parallax = self.get_parameter('reobs_min_parallax').value
        self.maha_threshold = self.get_parameter('maha_threshold').value
        self.cauchy_k = self.get_parameter('cauchy_k').value
        self.min_marker_depth = self.get_parameter('min_marker_depth').value
        self.max_marker_depth = self.get_parameter('max_marker_depth').value
        self.min_landmark_observations = int(
            self.get_parameter('min_landmark_observations').value
        )
        self.max_landmark_position_jump = float(
            self.get_parameter('max_landmark_position_jump').value
        )
        self.detection_keyframe_tolerance = float(
            self.get_parameter('detection_keyframe_tolerance').value
        )
        self.max_pending_detections = int(
            self.get_parameter('max_pending_detections').value
        )
        self.max_detection_wait_seconds = float(
            self.get_parameter('max_detection_wait_seconds').value
        )
        self.max_odom_buffer_samples = int(
            self.get_parameter('max_odom_buffer_samples').value
        )
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_extrinsics = CameraExtrinsics(
            tx=self.get_parameter('camera_tx').value,
            ty=self.get_parameter('camera_ty').value,
            yaw=self.get_parameter('camera_yaw').value,
        )
        self.geometry_debug_file = self.get_parameter('geometry_debug_file').value

        # ---------------- Estructuras del grafo ----------------
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial = gtsam.Values()
        self.result = None

        self.pose_count = 0
        self.seen_landmarks = set()
        self.landmark_gate = LandmarkObservationGate(self.min_landmark_observations)
        self.last_obs_pose = {}             # lm_id -> (x,y) del keyframe de ultima observacion
        self.lm_obs_count = {}             # lm_id -> número de factores añadidos
        self.geometry_observations = []

        self.last_odom_pose = None          # ultimo mensaje de odom (raw)
        self.last_kf_pose = None            # pose odom cruda del ultimo keyframe
        self.kf_world_pose = (0.0, 0.0, 0.0)
        self.kf_stamps: list = []           # timestamp (float s) de cada keyframe X(i)
        # Pose de odometria CRUDA (frame odom) en cada keyframe X(i). La 2a pasada
        # la usa para reconstruir la correccion SLAM (map<-odom) y aplicarla a la
        # odometria densa, evitando interpolar linealmente keyframes sparse.
        self.kf_odom_poses: list = []

        self.last_odom_stamp = None
        self.odom_samples = []
        self.dropped_stale_detections = 0
        self.dropped_pending_overflow = 0
        self._warned_camera_tf_fallback = False
        # Usamos reloj de pared para que el timer se dispare a 20 Hz
        # independientemente de si se está reproduciendo un bag con --clock.
        # Sin reloj de pared, con use_sim_time=true y sin --clock el timer
        # nunca se ejecuta porque el reloj sim queda congelado en 0.
        _wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.republish_tf, clock=_wall_clock)

        # ---------------- Modelos de ruido ----------------
        self.prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.01, 0.01, 0.005]))
        # Ruido odometría TB4: ~0.3m/0.15rad por keyframe permite que los
        # loop closures corrijan la deriva sin pelear contra un modelo demasiado rígido.
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.3, 0.3, 0.1]))

        # ---------------- TF broadcaster ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- Suscripciones / Publicaciones ----------------
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(Odometry, odom_topic, self.odom_cb, odom_qos)
        self.create_subscription(MarkerArray, lm_topic, self.landmarks_cb, 50)

        self.belief_pub = self.create_publisher(PoseStamped, '/belief', 10)
        self.poses_pub = self.create_publisher(PoseArray, '/poses_guardadas', 10)
        self.lm_pub = self.create_publisher(MarkerArray, optimized_landmarks_topic, 10)
        self.legacy_lm_pub = (
            self.create_publisher(MarkerArray, legacy_landmarks_topic, 10)
            if legacy_landmarks_topic and legacy_landmarks_topic != optimized_landmarks_topic
            else None
        )
        # Trayectoria corregida completa: la segunda pasada de mapeo LIDAR la consume
        self.path_pub = self.create_publisher(Path, '/trajectory_optimized', 10)
        self.base_debug_pub = self.create_publisher(MarkerArray, base_debug_topic, 10)

        self.pending_detections = []

        self.get_logger().info(
            f'graph_slam_node activo. odom={odom_topic} detecciones={lm_topic} '
            f'landmarks_optimizados={optimized_landmarks_topic} '
            f'aruco_base_debug={base_debug_topic} '
            f'kf_dist={self.kf_dist} kf_angle_max={self.kf_angle_max} '
            f'marker_depth=[{self.min_marker_depth}, {self.max_marker_depth}] '
            f'min_landmark_observations={self.min_landmark_observations} '
            f'max_landmark_position_jump={self.max_landmark_position_jump} '
            f'detection_keyframe_tolerance={self.detection_keyframe_tolerance} '
            f'reobs_min_parallax={self.reobs_min_parallax} '
            f'tf={self.map_frame}->{self.odom_frame}')

    # ============================================================
    #  Modelo de ruido de observacion dependiente de la distancia
    #  Rango: caracterizado con aruco_estimation (sigma_z(Z)=0.0239+0.0315*Z^2)
    #  Bearing: crece con la distancia (a mas lejos, angulo menos confiable)
    # ============================================================
    def obs_noise(self, range_):
        s_bearing, s_range = observation_sigmas(range_)
        return gtsam.noiseModel.Diagonal.Sigmas(np.array([s_bearing, s_range]))

    # ============================================================
    #  Callback de odometria: decide si crear un keyframe nuevo
    # ============================================================
    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        yaw = yaw_from_quaternion(p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w)
        odom_pose = (p.position.x, p.position.y, yaw)

        self.last_odom_stamp = msg.header.stamp
        odom_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._add_odom_sample(odom_stamp, odom_pose)

        if self.last_odom_pose is None:
            self.last_odom_pose = odom_pose
            self.last_kf_pose = odom_pose
            self._add_first_pose(odom_stamp)
            return

        self._process_ready_detections(newest_odom_stamp=odom_stamp)

        dx = odom_pose[0] - self.last_kf_pose[0]
        dy = odom_pose[1] - self.last_kf_pose[1]
        moved = math.hypot(dx, dy)
        turned = abs(normalize_angle(odom_pose[2] - self.last_kf_pose[2]))

        if moved >= self.kf_dist or turned >= self.kf_angle_max:
            pose_index = self._add_keyframe(
                self.last_kf_pose,
                odom_pose,
                odom_stamp,
                'motion_keyframe',
            )
            self._after_observations_added([], pose_index)

        self.last_odom_pose = odom_pose

    def republish_tf(self):
        """Republica el TF map->odom a 20 Hz usando el último stamp del bag.

        Al usar reloj de pared, se dispara aunque use_sim_time esté activo
        y el bag no envíe /clock. El stamp del TF es el último timestamp de
        odometría procesado: cubre el rango de tiempo del bag y permite que
        RViz interpole sin necesitar extrapolación.
        """
        if self.result is None or self.last_odom_stamp is None:
            return
        try:
            last = self.result.atPose2(X(self.pose_count - 1))
            self.publish_map_to_odom(last, self.last_odom_stamp)
        except Exception:
            pass

    def _add_odom_sample(self, stamp, odom_pose):
        stamp = float(stamp)
        sample = (stamp, tuple(odom_pose))
        stamps = [item[0] for item in self.odom_samples]
        index = bisect_left(stamps, stamp)
        if index < len(self.odom_samples) and self.odom_samples[index][0] == stamp:
            self.odom_samples[index] = sample
        else:
            self.odom_samples.insert(index, sample)
        overflow = len(self.odom_samples) - self.max_odom_buffer_samples
        if overflow > 0:
            del self.odom_samples[:overflow]

    def _odom_bracket_gap_ms(self, stamp):
        if not self.odom_samples:
            return None
        stamps = [item[0] for item in self.odom_samples]
        index = bisect_left(stamps, stamp)
        if index < len(self.odom_samples) and self.odom_samples[index][0] == stamp:
            return 0.0
        if index == 0 or index == len(self.odom_samples):
            return None
        return (self.odom_samples[index][0] - self.odom_samples[index - 1][0]) * 1000.0

    def _odom_at_detection(self, stamp):
        odom_pose = interpolate_bracketed(self.odom_samples, stamp)
        if odom_pose is None:
            return None, None
        return odom_pose, self._odom_bracket_gap_ms(stamp)

    def _add_first_pose(self, stamp):
        self.graph.add(gtsam.PriorFactorPose2(
            X(0), Pose2(0.0, 0.0, 0.0), self.prior_noise))
        self.initial.insert(X(0), Pose2(0.0, 0.0, 0.0))
        self.pose_count = 1
        self.kf_world_pose = (0.0, 0.0, 0.0)
        self.kf_stamps.append(float(stamp))
        self.kf_odom_poses.append(tuple(self.last_kf_pose))
        self.get_logger().info('Grafo inicializado: prior en X(0)=origen')

    def _add_keyframe(self, prev_odom_pose, curr_odom_pose, stamp, pose_source):
        """Crea un nuevo keyframe y conecta con BetweenFactor usando Pose2.between().

        Pose2.between() calcula T_prev^{-1} * T_curr (Lie group), la pose relativa
        exacta en el frame local de prev. Más robusto que la descomposición manual
        de deltas, especialmente para rotaciones grandes.
        """
        i = self.pose_count
        relative_pose = Pose2(*prev_odom_pose).between(Pose2(*curr_odom_pose))

        self.graph.add(gtsam.BetweenFactorPose2(
            X(i - 1), X(i), relative_pose, self.odom_noise))

        prev_est = self.initial.atPose2(X(i - 1))
        self.initial.insert(X(i), prev_est.compose(relative_pose))
        self.pose_count += 1
        self.kf_stamps.append(float(stamp))
        self.kf_odom_poses.append(tuple(curr_odom_pose))

        current_pose = self.initial.atPose2(X(i))
        self.kf_world_pose = (current_pose.x(), current_pose.y(), current_pose.theta())

        self.last_kf_pose = curr_odom_pose
        return i

    def _after_observations_added(self, observed_lm_ids, pose_index):
        if (self.pose_count % self.optimize_every) == 0:
            self.optimize()
            # Refresh last_obs_pose for this keyframe's landmarks using the
            # optimized pose, so the next parallax check uses corrected coords.
            if self.result is not None:
                try:
                    opt_pose = self.result.atPose2(X(pose_index))
                    opt_xy = (opt_pose.x(), opt_pose.y())
                    for lm_id in observed_lm_ids:
                        self.last_obs_pose[lm_id] = opt_xy
                except Exception:
                    pass
            self.publish_belief()
            self._flush_geometry_debug()

    def _nearest_pose_index(self, stamp):
        if not self.kf_stamps:
            return None, None
        index = bisect_left(self.kf_stamps, stamp)
        candidates = []
        if index < len(self.kf_stamps):
            candidates.append(index)
        if index > 0:
            candidates.append(index - 1)
        pose_index = min(candidates, key=lambda i: abs(self.kf_stamps[i] - stamp))
        return pose_index, self.kf_stamps[pose_index]

    def _pose_for_detection(self, pending):
        stamp = pending['stamp']
        pose_index, pose_stamp = self._nearest_pose_index(stamp)
        if pose_index is not None:
            dt = pose_stamp - stamp
            if abs(dt) <= self.detection_keyframe_tolerance:
                return pose_index, pose_stamp, 'existing_keyframe'
            if stamp < self.kf_stamps[-1]:
                return None, pose_stamp, 'dropped_stale'

        odom_pose = pending.get('odom_pose')
        if odom_pose is None:
            return None, None, 'waiting_odom'
        pose_index = self._add_keyframe(
            self.last_kf_pose,
            odom_pose,
            stamp,
            'created_detection_keyframe',
        )
        return pose_index, stamp, 'created_detection_keyframe'

    def _process_ready_detections(self, newest_odom_stamp):
        ready = []
        waiting = []
        for pending in self.pending_detections:
            odom_pose, gap_ms = self._odom_at_detection(pending['stamp'])
            if odom_pose is None:
                if newest_odom_stamp - pending['stamp'] > self.max_detection_wait_seconds:
                    self._record_dropped_observation(pending, 'dropped_stale')
                else:
                    waiting.append(pending)
                continue
            enriched = dict(pending)
            enriched['odom_pose'] = odom_pose
            enriched['odom_interpolation_gap_ms'] = gap_ms
            ready.append(enriched)
        self.pending_detections = waiting

        selected = {}
        pose_meta = {}
        for pending in sorted(ready, key=lambda item: item['stamp']):
            pose_index, pose_stamp, pose_source = self._pose_for_detection(pending)
            if pose_index is None:
                self._record_dropped_observation(
                    pending,
                    pose_source,
                    pose_index=self.pose_count - 1 if self.pose_count else None,
                    pose_stamp=pose_stamp,
                )
                continue
            pending = dict(pending)
            pending['pose_stamp'] = pose_stamp
            pending['detection_pose_dt'] = pose_stamp - pending['stamp']
            pending['pose_source'] = pose_source
            key = (pose_index, pending['id'])
            existing = selected.get(key)
            if existing is None or pending['range'] < existing['range']:
                selected[key] = pending
                pose_meta[pose_index] = pose_stamp

        observed_by_pose = {}
        for (pose_index, _marker_id), pending in selected.items():
            self._add_observation(pose_index, pending)
            observed_by_pose.setdefault(pose_index, []).append(pending['id'])
        for pose_index, observed_ids in observed_by_pose.items():
            self._after_observations_added(observed_ids, pose_index)

    # ============================================================
    #  Callback de detecciones ArUco
    # ============================================================
    def landmarks_cb(self, msg: MarkerArray):
        debug_markers = []
        for m in msg.markers:
            tx = m.pose.position.x
            ty = m.pose.position.y
            tz = m.pose.position.z
            if tz <= self.min_marker_depth or tz > self.max_marker_depth:
                continue
            x_base, y_base, tf_source = self._marker_to_base_xy(m, tx, ty, tz)
            rng = math.hypot(x_base, y_base)
            brg = math.atan2(y_base, x_base)
            pending = {
                'id': int(m.id),
                'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                'frame_id': m.header.frame_id,
                'tf_source': tf_source,
                'tx': float(tx),
                'ty': float(ty),
                'tz': float(tz),
                'x_base': float(x_base),
                'y_base': float(y_base),
                'range': float(rng),
                'bearing': float(brg),
            }
            debug_markers.append(pending)
            self._queue_detection(pending)

        if debug_markers:
            self.base_debug_pub.publish(
                build_base_debug_markers(
                    self.base_frame,
                    msg.markers[0].header.stamp,
                    debug_markers,
                )
            )

    def _queue_detection(self, pending):
        index = bisect_left(
            [item['stamp'] for item in self.pending_detections],
            pending['stamp'],
        )
        self.pending_detections.insert(index, pending)
        overflow = len(self.pending_detections) - self.max_pending_detections
        if overflow > 0:
            for stale in self.pending_detections[:overflow]:
                self._record_dropped_observation(stale, 'pending_overflow')
            del self.pending_detections[:overflow]
            self.dropped_pending_overflow += overflow

    def _marker_to_base_xy(self, marker, tx, ty, tz):
        """Convierte una detección en frame cámara a XY del robot.

        Primero intenta usar TF real desde el frame del marker hasta
        `base_frame`. Si no está disponible, usa los extrínsecos por
        parámetro como fallback.
        """
        frame_id = marker.header.frame_id if marker.header.frame_id else ''
        stamp = marker.header.stamp

        if frame_id:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    frame_id,
                    stamp,
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
                x_base, y_base = transform_stamped_to_base_xy(tf, (tx, ty, tz))
                return x_base, y_base, 'tf'
            except Exception:
                pass

        if not self._warned_camera_tf_fallback:
            self.get_logger().warning(
                'No se pudo resolver TF camara->base_link; usando extrinsecos fallback '
                f'(tx={self.camera_extrinsics.tx}, ty={self.camera_extrinsics.ty}, '
                f'yaw={self.camera_extrinsics.yaw}).'
            )
            self._warned_camera_tf_fallback = True
        x_base, y_base = fallback_camera_to_base_xy(
            tx=tx,
            tz=tz,
            extrinsics=self.camera_extrinsics,
        )
        return x_base, y_base, 'fallback'

    MAX_OBS_PER_LANDMARK = 50

    def _innovation_gate(self, pose_index, lm_id, bearing, range_):
        """Filtra re-observaciones por distancia de Mahalanobis.

        Compara la observación (bearing, range) con la predicción del estado
        actual. Rechaza si maha² > maha_threshold (chi2 2-DOF, 95% → 5.99).
        Solo actúa cuando hay un resultado optimizado disponible.
        """
        state = resolve_gate_state(
            result=self.result,
            initial=self.initial,
            pose_key=X(pose_index),
            landmark_key=L(lm_id),
        )
        if state is None:
            return True
        pose, lm = state
        maha2 = innovation_mahalanobis_sq(
            pose=pose,
            landmark=lm,
            bearing=bearing,
            range_=range_,
        )

        if maha2 >= self.maha_threshold:
            self.get_logger().info(
                f'Gating: L({lm_id}) rechazado maha²={maha2:.2f} '
                f'(umbral {self.maha_threshold})')
            return False
        return True

    def _robust_obs_noise(self, range_):
        """Ruido de observación envuelto en kernel robusto de Cauchy.

        El kernel Cauchy satura para residuos grandes (outliers) en lugar de
        crecer cuadráticamente, evitando que una mala re-observación distorsione
        toda la trayectoria. cauchy_k controla el punto de transición en
        unidades de residuo whitened.
        """
        base = self.obs_noise(range_)
        try:
            robust = gtsam.noiseModel.Robust.Create(
                gtsam.noiseModel.mEstimator.Cauchy.Create(self.cauchy_k),
                base,
            )
            return robust
        except Exception:
            return base

    def _record_dropped_observation(self, pending, reason, pose_index=None, pose_stamp=None):
        if pose_index is None:
            pose_index = self.pose_count - 1 if self.pose_count else 0
        debug_pose = self._pose_for_debug(pose_index)
        if pose_stamp is None and pose_index < len(self.kf_stamps):
            pose_stamp = self.kf_stamps[pose_index]
        self.geometry_observations.append(ArucoGeometryObservation(
            stamp=pending['stamp'],
            marker_id=pending['id'],
            frame_id=pending['frame_id'],
            tf_source=pending['tf_source'],
            tx=pending['tx'],
            ty=pending['ty'],
            tz=pending['tz'],
            x_base=pending['x_base'],
            y_base=pending['y_base'],
            range_=pending['range'],
            bearing=pending['bearing'],
            pose_index=pose_index,
            pose_x=debug_pose.x(),
            pose_y=debug_pose.y(),
            pose_theta=debug_pose.theta(),
            detection_stamp=pending['stamp'],
            pose_stamp=pose_stamp,
            detection_pose_dt=(pose_stamp - pending['stamp']) if pose_stamp is not None else None,
            pose_source='dropped_stale' if reason == 'dropped_stale' else reason,
            odom_interpolation_gap_ms=pending.get('odom_interpolation_gap_ms'),
            reject_reason=reason,
        ))
        self.dropped_stale_detections += int(reason == 'dropped_stale')
        self._flush_geometry_debug()

    def _add_observation(self, pose_index, pending):
        landmark_id = pending['id']
        range_ = pending['range']
        bearing = pending['bearing']
        px, py = self._robot_xy_for_pose(pose_index)
        debug_pose = self._pose_for_debug(pose_index)
        geometry_observation = ArucoGeometryObservation(
            stamp=pending['stamp'],
            marker_id=landmark_id,
            frame_id=pending['frame_id'],
            tf_source=pending['tf_source'],
            tx=pending['tx'],
            ty=pending['ty'],
            tz=pending['tz'],
            x_base=pending['x_base'],
            y_base=pending['y_base'],
            range_=range_,
            bearing=bearing,
            pose_index=pose_index,
            pose_x=debug_pose.x(),
            pose_y=debug_pose.y(),
            pose_theta=debug_pose.theta(),
            detection_stamp=pending.get('stamp'),
            pose_stamp=pending.get('pose_stamp'),
            detection_pose_dt=pending.get('detection_pose_dt'),
            pose_source=pending.get('pose_source', ''),
            odom_interpolation_gap_ms=pending.get('odom_interpolation_gap_ms'),
        )
        self.geometry_observations.append(geometry_observation)

        if landmark_id in self.seen_landmarks:
            lox, loy = self.last_obs_pose[landmark_id]
            if math.hypot(px - lox, py - loy) < self.reobs_min_parallax:
                return
            if self.lm_obs_count.get(landmark_id, 0) >= self.MAX_OBS_PER_LANDMARK:
                return
            spatial_gate = spatial_landmark_gate_from_values(
                result=self.result,
                initial=self.initial,
                pose_key=X(pose_index),
                landmark_key=L(landmark_id),
                x_base=pending['x_base'],
                y_base=pending['y_base'],
                max_jump=self.max_landmark_position_jump,
            )
            spatial_ok, pred_x, pred_y, jump, _, _ = spatial_gate
            if pred_x is not None and pred_y is not None:
                geometry_observation = replace(
                    geometry_observation,
                    predicted_landmark_x=pred_x,
                    predicted_landmark_y=pred_y,
                    spatial_jump=jump,
                )
                self.geometry_observations[-1] = geometry_observation
            if not spatial_ok:
                geometry_observation = replace(
                    geometry_observation,
                    reject_reason='spatial_jump',
                )
                self.geometry_observations[-1] = geometry_observation
                self.get_logger().info(
                    f'Gating espacial: L({landmark_id}) rechazado '
                    f'spatial_jump={jump:.3f} m '
                    f'(umbral {self.max_landmark_position_jump:.3f} m)')
                self._flush_geometry_debug()
                return
            if not self._innovation_gate(pose_index, landmark_id, bearing, range_):
                return
            self.get_logger().info(f'LOOP CLOSURE: re-observado L({landmark_id})')
        else:
            accepted, count = self.landmark_gate.register(landmark_id)
            if not accepted:
                self.get_logger().info(
                    f'Candidato L({landmark_id}) observado {count}/'
                    f'{self.min_landmark_observations}; no se inicializa todavia')
                self._flush_geometry_debug()
                return
            pose_est = self.initial.atPose2(X(pose_index))
            lx = pose_est.x() + range_ * math.cos(pose_est.theta() + bearing)
            ly = pose_est.y() + range_ * math.sin(pose_est.theta() + bearing)
            self.initial.insert(L(landmark_id), Point2(lx, ly))
            self.seen_landmarks.add(landmark_id)
            self.landmark_gate.clear(landmark_id)
            self.lm_obs_count[landmark_id] = 0
            self.get_logger().info(f'Nuevo landmark L({landmark_id}) inicializado')

        self.graph.add(gtsam.BearingRangeFactor2D(
            X(pose_index), L(landmark_id),
            Rot2.fromAngle(bearing), range_,
            self._robust_obs_noise(range_),
        ))
        self.lm_obs_count[landmark_id] = self.lm_obs_count.get(landmark_id, 0) + 1
        self.last_obs_pose[landmark_id] = (px, py)
        self._flush_geometry_debug()


    def _robot_xy_for_pose(self, pose_index):
        values = self.result if self.result is not None else self.initial
        try:
            pose = values.atPose2(X(pose_index))
            return pose.x(), pose.y()
        except Exception:
            return self.kf_world_pose[0], self.kf_world_pose[1]
    def _current_robot_xy(self):
        """Devuelve la mejor estimación disponible de la posición actual."""
        if self.result is not None and self.pose_count > 0:
            try:
                pose = self.result.atPose2(X(self.pose_count - 1))
                return pose.x(), pose.y()
            except Exception:
                pass
        return self.kf_world_pose[0], self.kf_world_pose[1]

    def _pose_for_debug(self, pose_index):
        values = self.result if self.result is not None else self.initial
        try:
            return values.atPose2(X(pose_index))
        except Exception:
            return Pose2(*self.kf_world_pose)

    def _values_for_geometry_debug(self):
        return self.result if self.result is not None else self.initial

    def _landmark_xy_for_debug(self, values, landmark_id):
        if landmark_id not in self.seen_landmarks:
            return None, None
        try:
            point = values.atPoint2(L(landmark_id))
            return float(point[0]), float(point[1])
        except Exception:
            return None, None

    def _flush_geometry_debug(self):
        if not self.geometry_debug_file:
            return
        values = self._values_for_geometry_debug()
        rows = []
        for observation in self.geometry_observations:
            observation_for_row = observation
            try:
                pose = values.atPose2(X(observation.pose_index))
                observation_for_row = ArucoGeometryObservation(
                    stamp=observation.stamp,
                    marker_id=observation.marker_id,
                    frame_id=observation.frame_id,
                    tf_source=observation.tf_source,
                    tx=observation.tx,
                    ty=observation.ty,
                    tz=observation.tz,
                    x_base=observation.x_base,
                    y_base=observation.y_base,
                    range_=observation.range_,
                    bearing=observation.bearing,
                    pose_index=observation.pose_index,
                    pose_x=pose.x(),
                    pose_y=pose.y(),
                    pose_theta=pose.theta(),
                    detection_stamp=observation.detection_stamp,
                    pose_stamp=observation.pose_stamp,
                    detection_pose_dt=observation.detection_pose_dt,
                    pose_source=observation.pose_source,
                    odom_interpolation_gap_ms=observation.odom_interpolation_gap_ms,
                    predicted_landmark_x=observation.predicted_landmark_x,
                    predicted_landmark_y=observation.predicted_landmark_y,
                    spatial_jump=observation.spatial_jump,
                    reject_reason=observation.reject_reason,
                )
            except Exception:
                pass
            landmark_x, landmark_y = self._landmark_xy_for_debug(
                values,
                observation.marker_id,
            )
            rows.append(build_geometry_debug_row(observation_for_row, landmark_x, landmark_y))
        try:
            parent_dir = os.path.dirname(self.geometry_debug_file)
            if parent_dir:
                os.makedirs(parent_dir, exist_ok=True)
            with open(self.geometry_debug_file, 'w', newline='') as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=[
                        'stamp',
                        'id',
                        'frame_id',
                        'tf_source',
                        'tx',
                        'ty',
                        'tz',
                        'x_base',
                        'y_base',
                        'range',
                        'bearing',
                        'pose_index',
                        'detection_stamp',
                        'pose_stamp',
                        'detection_pose_dt',
                        'pose_source',
                        'odom_interpolation_gap_ms',
                        'pose_x',
                        'pose_y',
                        'pose_theta',
                        'landmark_x',
                        'landmark_y',
                        'residual_range',
                        'predicted_landmark_x',
                        'predicted_landmark_y',
                        'spatial_jump',
                        'reject_reason',
                    ],
                )
                writer.writeheader()
                writer.writerows(rows)
        except Exception as exc:
            self.get_logger().warn(f'No se pudo escribir diagnostico geometrico ArUco: {exc}')

    # ============================================================
    #  Optimizacion
    # ============================================================
    def optimize(self):
        if self.pose_count < 2:
            return
        try:
            self.result, self.initial, err0, err1 = optimize_graph(
                graph=self.graph,
                initial=self.initial,
                pose_count=self.pose_count,
                landmark_ids=list(self.seen_landmarks),
            )
        except Exception as exc:
            self.get_logger().error(f'optimize() fallo: {exc}')
            return
        self.get_logger().info(
            f'Optimizado: {self.pose_count} poses, '
            f'{len(self.seen_landmarks)} landmarks. '
            f'error {err0:.3f} -> {err1:.3f}')
        self._flush_geometry_debug()

    # ============================================================
    #  Publicacion para RViz + TF map->odom
    # ============================================================
    def publish_belief(self):
        if self.result is None:
            return

        now = self.get_clock().now().to_msg()
        poses = [self.result.atPose2(X(i)) for i in range(self.pose_count)]
        last = poses[-1]

        self.belief_pub.publish(build_belief_message(self.map_frame, now, last))
        self.poses_pub.publish(build_pose_array_message(self.map_frame, now, poses))
        self.path_pub.publish(build_path_message(self.map_frame, now, poses))

        landmarks = []
        for lm_id in self.seen_landmarks:
            try:
                landmarks.append((lm_id, self.result.atPoint2(L(lm_id))))
            except Exception:
                pass
        landmark_msg = build_landmark_markers(self.map_frame, now, landmarks)
        self.lm_pub.publish(landmark_msg)
        if self.legacy_lm_pub is not None:
            self.legacy_lm_pub.publish(landmark_msg)

        # TF map->odom
        tf_stamp = self.last_odom_stamp if self.last_odom_stamp is not None else now
        self.publish_map_to_odom(last, tf_stamp)

    def publish_map_to_odom(self, map_base_pose, stamp):
        """
        map->odom = (map->base) * (odom->base)^-1
        map->base  = pose optimizada del robot (en 'map')
        odom->base = pose odom cruda del mismo keyframe (en 'odom')
        """
        ox, oy, oth = self.last_kf_pose
        T_odom_base = Pose2(ox, oy, oth)
        T_map_odom = map_base_pose.compose(T_odom_base.inverse())

        self.tf_broadcaster.sendTransform(
            build_map_to_odom_transform(
                self.map_frame,
                self.odom_frame,
                stamp,
                T_map_odom,
            )
        )

    # ============================================================
    #  Exportación de trayectoria y landmarks al finalizar
    # ============================================================
    def save_trajectory(self):
        """Guarda trayectoria optimizada + posiciones de landmarks a JSON.

        El archivo generado es la entrada para la segunda pasada de mapeo LIDAR
        (occupancy_grid_node, Fase 5): contiene las poses corregidas que permiten
        proyectar cada barrido del LIDAR sin deriva acumulada.
        """
        path = self.get_parameter('trajectory_file').value
        if not path:
            return
        values = self.result
        if values is None:
            if self.pose_count == 0:
                self.get_logger().warning(
                    'trajectory_file configurado pero no hay poses para exportar.')
                return
            values = self.initial
            self.get_logger().warning(
                'trajectory_file configurado pero no hay resultado optimizado; '
                'exportando la estimacion inicial.')

        traj = []
        for i in range(self.pose_count):
            try:
                p = values.atPose2(X(i))
                entry = {
                    'i': i, 'x': p.x(), 'y': p.y(), 'theta': p.theta(),
                    'stamp': self.kf_stamps[i] if i < len(self.kf_stamps) else 0.0,
                }
                if i < len(self.kf_odom_poses):
                    ox, oy, oth = self.kf_odom_poses[i]
                    entry['odom'] = {'x': ox, 'y': oy, 'theta': oth}
                traj.append(entry)
            except Exception:
                pass

        lm_data = {}
        for lm_id in self.seen_landmarks:
            try:
                pt = values.atPoint2(L(lm_id))
                lm_data[str(lm_id)] = {'x': float(pt[0]), 'y': float(pt[1])}
            except Exception:
                pass

        write_trajectory_json(path, traj, lm_data)
        self._flush_geometry_debug()
        self.get_logger().info(
            f'Trayectoria guardada: {len(traj)} poses, '
            f'{len(lm_data)} landmarks → {path}')


def main(args=None):
    rclpy.init(args=args)
    node = GraphSlamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_trajectory()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
