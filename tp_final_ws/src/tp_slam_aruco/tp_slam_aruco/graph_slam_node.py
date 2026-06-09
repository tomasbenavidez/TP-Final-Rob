import bisect
import math
import numpy as np

import gtsam
from gtsam import Point2, Pose2, Rot2
from gtsam.symbol_shorthand import L, X

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray

from tp_slam_interfaces.msg import LandmarkObservationArray, VisualObservability
from tp_slam_aruco.motion_model import normalize_angle, yaw_from_quaternion
from tp_slam_aruco.slam_gating import (
    classify_innovation_diagnostics,
    innovation_diagnostics,
    innovation_mahalanobis_sq,
    observation_sigmas,
    resolve_gate_state,
)
from tp_slam_aruco.slam_graph_diagnostics import GraphRunDiagnostics
from tp_slam_aruco.slam_graph import optimize_graph
from tp_slam_aruco.slam_io import write_trajectory_json
from tp_slam_aruco.slam_landmarks import LandmarkCandidateManager
from tp_slam_aruco.slam_publish import (
    build_belief_message,
    build_landmark_markers,
    build_map_to_odom_transform,
    build_path_message,
    build_pose_array_message,
)
from tp_slam_aruco.slam_timing import (
    OdomPoseBuffer,
    TimedPose2,
    observation_requires_new_keyframe,
    should_create_visual_keyframe,
)
from tp_slam_aruco.visual_observability import FrameObservability


class GraphSlamNode(Node):
    MAX_OBS_PER_LANDMARK = 20

    def __init__(self):
        super().__init__('graph_slam_node')

        self.declare_parameter('odom_topic', 'tb4_0/odom')
        self.declare_parameter('observations_topic', '/landmark_observations')
        self.declare_parameter('kf_dist', 0.15)
        self.declare_parameter('optimize_every', 1)
        self.declare_parameter('kf_angle_max', 0.60)
        self.declare_parameter('reobs_min_parallax', 0.12)
        self.declare_parameter('maha_threshold', 5.99)
        self.declare_parameter('cauchy_k', 1.0)
        self.declare_parameter('min_visual_landmarks', 2)
        self.declare_parameter('min_candidate_observations', 2)
        self.declare_parameter('max_candidate_reprojection_error_px', 4.0)
        self.declare_parameter('diagnostics_every_batches', 25)
        self.declare_parameter('max_candidate_buffer_observations', 10)
        self.declare_parameter('candidate_reset_on_bad_observation', True)
        self.declare_parameter('min_candidate_pose_separation_m', 0.08)
        self.declare_parameter('max_observation_age_s_for_new_landmark', 0.20)
        self.declare_parameter('max_observation_age_s_for_reobservation', 0.25)
        self.declare_parameter('min_landmark_clearance_m', 0.20)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_tx', 0.0)
        self.declare_parameter('camera_ty', 0.0)
        self.declare_parameter('camera_yaw', 0.0)
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('visual_diagnostics_topic', '/landmark_detection_stats')

        odom_topic = self.get_parameter('odom_topic').value
        observations_topic = self.get_parameter('observations_topic').value
        visual_diagnostics_topic = self.get_parameter('visual_diagnostics_topic').value
        self.kf_dist = float(self.get_parameter('kf_dist').value)
        self.optimize_every = int(self.get_parameter('optimize_every').value)
        self.kf_angle_max = float(self.get_parameter('kf_angle_max').value)
        self.reobs_min_parallax = float(self.get_parameter('reobs_min_parallax').value)
        self.maha_threshold = float(self.get_parameter('maha_threshold').value)
        self.cauchy_k = float(self.get_parameter('cauchy_k').value)
        self.min_visual_landmarks = int(self.get_parameter('min_visual_landmarks').value)
        self.min_candidate_observations = int(
            self.get_parameter('min_candidate_observations').value
        )
        self.max_candidate_reprojection_error_px = float(
            self.get_parameter('max_candidate_reprojection_error_px').value
        )
        self.diagnostics_every_batches = int(
            self.get_parameter('diagnostics_every_batches').value
        )
        self.max_candidate_buffer_observations = int(
            self.get_parameter('max_candidate_buffer_observations').value
        )
        self.candidate_reset_on_bad_observation = bool(
            self.get_parameter('candidate_reset_on_bad_observation').value
        )
        self.min_candidate_pose_separation_m = float(
            self.get_parameter('min_candidate_pose_separation_m').value
        )
        self.max_observation_age_s_for_new_landmark = float(
            self.get_parameter('max_observation_age_s_for_new_landmark').value
        )
        self.max_observation_age_s_for_reobservation = float(
            self.get_parameter('max_observation_age_s_for_reobservation').value
        )
        self.min_landmark_clearance_m = float(
            self.get_parameter('min_landmark_clearance_m').value
        )
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.graph = gtsam.NonlinearFactorGraph()
        self.initial = gtsam.Values()
        self.result = None

        self.pose_count = 0
        self.seen_landmarks = set()
        self.last_obs_pose = {}
        self.lm_obs_count = {}

        self.odom_buffer = OdomPoseBuffer()
        self.last_odom_stamp = None
        self.last_odom_timed_pose = None
        self.last_kf_timed_pose = None
        self.last_kf_pose = None
        self.kf_stamps = []
        self._last_obs_age_warning_stamp = None
        self._observation_batch_count = 0

        self.diagnostics = GraphRunDiagnostics()
        self.candidate_manager = LandmarkCandidateManager(
            min_candidate_observations=self.min_candidate_observations,
            min_candidate_parallax_m=self.reobs_min_parallax,
            max_candidate_reprojection_error_px=self.max_candidate_reprojection_error_px,
            max_candidate_buffer_observations=self.max_candidate_buffer_observations,
            candidate_reset_on_bad_observation=self.candidate_reset_on_bad_observation,
            min_candidate_pose_separation_m=self.min_candidate_pose_separation_m,
        )

        self.prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.01, 0.01, 0.005])
        )
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.3, 0.3, 0.1])
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.republish_tf, clock=wall_clock)

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(Odometry, odom_topic, self.odom_cb, odom_qos)
        self.create_subscription(
            LandmarkObservationArray,
            observations_topic,
            self.observations_cb,
            50,
        )
        self.create_subscription(
            VisualObservability,
            visual_diagnostics_topic,
            self.visual_observability_cb,
            50,
        )

        self.belief_pub = self.create_publisher(PoseStamped, '/belief', 10)
        self.poses_pub = self.create_publisher(PoseArray, '/poses_guardadas', 10)
        self.lm_pub = self.create_publisher(MarkerArray, '/landmarks_opt', 10)
        self.path_pub = self.create_publisher(Path, '/trajectory_optimized', 10)

        self.get_logger().info(
            f'graph_slam_node activo. odom={odom_topic} observations={observations_topic} '
            f'kf_dist={self.kf_dist} kf_angle_max={self.kf_angle_max} '
            f'reobs_min_parallax={self.reobs_min_parallax} '
            f'min_visual_landmarks={self.min_visual_landmarks} '
            f'visual_diag={visual_diagnostics_topic} '
            f'qos(odom)=BEST_EFFORT tf={self.map_frame}->{self.odom_frame}'
        )

    def obs_noise(self, range_):
        s_bearing, s_range = observation_sigmas(range_)
        return gtsam.noiseModel.Diagonal.Sigmas(np.array([s_bearing, s_range]))

    def odom_cb(self, msg):
        timed_pose = self._timed_pose_from_odom(msg)
        self.last_odom_stamp = msg.header.stamp
        self.last_odom_timed_pose = timed_pose
        self.odom_buffer.append(timed_pose)

        if self.last_kf_timed_pose is None:
            self.last_kf_timed_pose = timed_pose
            self.last_kf_pose = (timed_pose.x, timed_pose.y, timed_pose.theta)
            self._add_first_pose(timed_pose.stamp)
            return

        if observation_requires_new_keyframe(
            last_keyframe=self.last_kf_timed_pose,
            observation_pose=timed_pose,
            kf_dist=self.kf_dist,
            kf_angle_max=self.kf_angle_max,
        ):
            self._add_keyframe(self.last_kf_timed_pose, timed_pose)

    def observations_cb(self, msg):
        if self.pose_count == 0:
            return

        latest_odom = self.odom_buffer.latest()
        if latest_odom is None:
            return

        selected = {}
        factor_added = False
        used_landmark_count = 0

        for observation in msg.observations:
            obs_stamp = self._stamp_to_seconds(observation.header.stamp)
            obs_pose = self.odom_buffer.pose_at(obs_stamp)
            if obs_pose is None:
                continue

            obs_age = latest_odom.stamp - obs_stamp
            if obs_age > 0.2 and self._last_obs_age_warning_stamp != obs_stamp:
                self.get_logger().info(
                    f'Observación de L({observation.landmark_id}) llega con edad '
                    f'{obs_age:.3f}s respecto a odom.'
                )
                self._last_obs_age_warning_stamp = obs_stamp

            pose_index = self._resolve_pose_index_for_observation(
                obs_pose,
                valid_landmark_count=len(msg.observations),
            )
            key = (pose_index, int(observation.landmark_id))
            existing = selected.get(key)
            if existing is None or observation.range_m < existing.range_m:
                selected[key] = observation

        self._observation_batch_count += 1
        for (pose_index, landmark_id), observation in sorted(selected.items()):
            observation_age_s = max(
                0.0,
                latest_odom.stamp - self._stamp_to_seconds(observation.header.stamp),
            )
            observation_added = self._add_observation(
                pose_index=pose_index,
                landmark_id=landmark_id,
                range_=float(observation.range_m),
                bearing=float(observation.bearing_rad),
                reprojection_error_px=float(observation.reprojection_error_px),
                pose_xy=self._pose_xy_for_index(pose_index),
                projected_xy=self._project_landmark_xy(
                    pose_index=pose_index,
                    bearing=float(observation.bearing_rad),
                    range_=float(observation.range_m),
                ),
                observation_age_s=observation_age_s,
            )
            factor_added = observation_added or factor_added
            if observation_added:
                used_landmark_count += 1

        self.diagnostics.record_observation_batch(
            valid_landmark_count=len(selected),
            used_landmark_count=used_landmark_count,
        )
        if self._observation_batch_count % self.diagnostics_every_batches == 0:
            self._log_diagnostics_summary()

        if factor_added and self.pose_count >= 2:
            self.optimize()
            self.publish_belief()

    def republish_tf(self):
        if self.result is None or self.last_odom_stamp is None or self.pose_count == 0:
            return
        try:
            last = self.result.atPose2(X(self.pose_count - 1))
            self.publish_map_to_odom(last, self.last_odom_stamp)
        except Exception:
            pass

    def visual_observability_cb(self, msg):
        self.diagnostics.record_visual_observability(
            frame=FrameObservability(
                stamp=self._stamp_to_seconds(msg.header.stamp),
                raw_count=int(msg.raw_count),
                valid_count=int(msg.valid_count),
                valid_unique_count=int(msg.valid_unique_count),
                rejected_area=int(msg.rejected_area),
                rejected_depth=int(msg.rejected_depth),
                rejected_reprojection=int(msg.rejected_reprojection),
                rejected_tf=int(msg.rejected_tf),
                rejected_no_calibration=int(msg.rejected_no_calibration),
                raw_ids=tuple(int(x) for x in msg.raw_ids),
                valid_ids=tuple(int(x) for x in msg.valid_ids),
                rejected_area_ids=tuple(int(x) for x in msg.rejected_area_ids),
                rejected_depth_ids=tuple(int(x) for x in msg.rejected_depth_ids),
                rejected_reprojection_ids=tuple(int(x) for x in msg.rejected_reprojection_ids),
                rejected_tf_ids=tuple(int(x) for x in msg.rejected_tf_ids),
            )
        )

    def _add_first_pose(self, stamp_sec):
        self.graph.add(gtsam.PriorFactorPose2(
            X(0), Pose2(0.0, 0.0, 0.0), self.prior_noise
        ))
        self.initial.insert(X(0), Pose2(0.0, 0.0, 0.0))
        self.pose_count = 1
        self.kf_stamps.append(stamp_sec)
        self.diagnostics.record_keyframe()
        self.get_logger().info('Grafo inicializado: prior en X(0)=origen')

    def _add_keyframe(self, prev_pose, curr_pose):
        i = self.pose_count
        relative_pose = Pose2(prev_pose.x, prev_pose.y, prev_pose.theta).between(
            Pose2(curr_pose.x, curr_pose.y, curr_pose.theta)
        )

        self.graph.add(gtsam.BetweenFactorPose2(
            X(i - 1), X(i), relative_pose, self.odom_noise
        ))

        prev_est = self.initial.atPose2(X(i - 1))
        self.initial.insert(X(i), prev_est.compose(relative_pose))
        self.pose_count += 1
        self.kf_stamps.append(curr_pose.stamp)
        self.diagnostics.record_keyframe()

        self.last_kf_timed_pose = curr_pose
        self.last_kf_pose = (curr_pose.x, curr_pose.y, curr_pose.theta)

        if (self.pose_count % self.optimize_every) == 0 and self.pose_count >= 2:
            self.optimize()
            self.publish_belief()

    def _resolve_pose_index_for_observation(self, obs_pose, valid_landmark_count):
        if (
            self.last_kf_timed_pose is not None
            and obs_pose.stamp >= self.last_kf_timed_pose.stamp
            and should_create_visual_keyframe(
                last_keyframe=self.last_kf_timed_pose,
                observation_pose=obs_pose,
                kf_dist=self.kf_dist,
                kf_angle_max=self.kf_angle_max,
                valid_landmark_count=valid_landmark_count,
                min_visual_landmarks=self.min_visual_landmarks,
            )
        ):
            self._add_keyframe(self.last_kf_timed_pose, obs_pose)
            return self.pose_count - 1

        return self._keyframe_index_for_stamp(obs_pose.stamp)

    def _keyframe_index_for_stamp(self, stamp):
        index = bisect.bisect_right(self.kf_stamps, stamp) - 1
        return max(0, min(index, self.pose_count - 1))

    def _innovation_gate(self, pose_index, lm_id, bearing, range_, reprojection_error_px):
        state = resolve_gate_state(
            result=self.result,
            initial=self.initial,
            pose_key=X(pose_index),
            landmark_key=L(lm_id),
        )
        if state is None:
            return True

        pose, landmark = state
        diag = innovation_diagnostics(
            pose=pose,
            landmark=landmark,
            bearing=bearing,
            range_=range_,
        )
        if diag.maha_sq >= self.maha_threshold:
            classification = classify_innovation_diagnostics(diag)
            self.diagnostics.record_gating_rejection(
                lm_id,
                diag.maha_sq,
                classification,
            )
            self.get_logger().info(
                f'Gating: L({lm_id}) rechazado maha²={diag.maha_sq:.2f} '
                f'(umbral {self.maha_threshold}) '
                f'dr={diag.range_residual:.3f} db={diag.bearing_residual:.3f} '
                f'pred_r={diag.pred_range:.3f} pred_b={diag.pred_bearing:.3f} '
                f'reproj={reprojection_error_px:.2f}'
            )
            return False
        self.diagnostics.record_gating_acceptance()
        return True

    def _robust_obs_noise(self, range_):
        base = self.obs_noise(range_)
        try:
            return gtsam.noiseModel.Robust.Create(
                gtsam.noiseModel.mEstimator.Cauchy.Create(self.cauchy_k),
                base,
            )
        except Exception:
            return base

    def _add_observation(
        self,
        pose_index,
        landmark_id,
        range_,
        bearing,
        reprojection_error_px,
        pose_xy,
        projected_xy,
        observation_age_s,
    ):
        px, py = self._pose_xy_for_index(pose_index)

        if landmark_id in self.seen_landmarks:
            if observation_age_s > self.max_observation_age_s_for_reobservation:
                self.diagnostics.record_age_discard(landmark_id, for_new_landmark=False)
                return False
            lox, loy = self.last_obs_pose[landmark_id]
            if math.hypot(px - lox, py - loy) < self.reobs_min_parallax:
                return False
            if self.lm_obs_count.get(landmark_id, 0) >= self.MAX_OBS_PER_LANDMARK:
                return False
            if not self._innovation_gate(
                pose_index,
                landmark_id,
                bearing,
                range_,
                reprojection_error_px,
            ):
                return False
            self.diagnostics.record_loop_closure()
            self.get_logger().info(f'LOOP CLOSURE: re-observado L({landmark_id})')
        else:
            candidate = self.candidate_manager.observe(
                landmark_id=landmark_id,
                pose_xy=pose_xy,
                projected_xy=projected_xy,
                reprojection_error_px=reprojection_error_px,
                observation_age_s=observation_age_s,
                max_observation_age_s=self.max_observation_age_s_for_new_landmark,
            )
            if candidate is None:
                if observation_age_s > self.max_observation_age_s_for_new_landmark:
                    self.diagnostics.record_age_discard(landmark_id, for_new_landmark=True)
                self.diagnostics.record_candidate()
                return False
            self.initial.insert(
                L(landmark_id),
                Point2(candidate.projected_x, candidate.projected_y),
            )
            self.seen_landmarks.add(landmark_id)
            self.lm_obs_count[landmark_id] = 0
            self.diagnostics.record_seeded_landmark(landmark_id)
            self.diagnostics.record_confirmed_landmark(landmark_id)
            self.get_logger().info(
                f'Nuevo landmark L({landmark_id}) confirmado '
                f'con {candidate.observation_count} observaciones'
            )

        self.graph.add(gtsam.BearingRangeFactor2D(
            X(pose_index),
            L(landmark_id),
            Rot2.fromAngle(bearing),
            range_,
            self._robust_obs_noise(range_),
        ))
        self.lm_obs_count[landmark_id] = self.lm_obs_count.get(landmark_id, 0) + 1
        self.last_obs_pose[landmark_id] = (px, py)
        return True

    def _pose_for_index(self, pose_index):
        if self.result is not None:
            try:
                return self.result.atPose2(X(pose_index))
            except Exception:
                pass
        return self.initial.atPose2(X(pose_index))

    def _project_landmark_xy(self, pose_index, bearing, range_):
        pose_est = self._pose_for_index(pose_index)
        lx = pose_est.x() + range_ * math.cos(pose_est.theta() + bearing)
        ly = pose_est.y() + range_ * math.sin(pose_est.theta() + bearing)
        return lx, ly

    def _pose_xy_for_index(self, pose_index):
        pose = self._pose_for_index(pose_index)
        return pose.x(), pose.y()

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
        self.diagnostics.record_optimization(err0, err1)
        self.get_logger().info(
            f'Optimizado: {self.pose_count} poses, '
            f'{len(self.seen_landmarks)} landmarks. '
            f'error {err0:.3f} -> {err1:.3f}'
        )

    def publish_belief(self):
        if self.result is None or self.pose_count == 0:
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
        self.lm_pub.publish(build_landmark_markers(self.map_frame, now, landmarks))

        tf_stamp = self.last_odom_stamp if self.last_odom_stamp is not None else now
        self.publish_map_to_odom(last, tf_stamp)

    def publish_map_to_odom(self, map_base_pose, stamp):
        if self.last_kf_pose is None:
            return
        ox, oy, oth = self.last_kf_pose
        t_odom_base = Pose2(ox, oy, oth)
        t_map_odom = map_base_pose.compose(t_odom_base.inverse())

        self.tf_broadcaster.sendTransform(
            build_map_to_odom_transform(
                self.map_frame,
                self.odom_frame,
                stamp,
                t_map_odom,
            )
        )

    def save_trajectory(self):
        path = self.get_parameter('trajectory_file').value
        if not path or self.result is None:
            if path and self.result is None:
                self.get_logger().warning(
                    'trajectory_file configurado pero no hay resultado optimizado. '
                    'El bag terminó antes de crear keyframes suficientes.'
                )
            return

        traj = []
        for i in range(self.pose_count):
            try:
                pose = self.result.atPose2(X(i))
                traj.append({
                    'i': i,
                    'x': pose.x(),
                    'y': pose.y(),
                    'theta': pose.theta(),
                    'stamp': self.kf_stamps[i] if i < len(self.kf_stamps) else 0.0,
                })
            except Exception:
                pass

        lm_data = {}
        for lm_id in self.seen_landmarks:
            try:
                point = self.result.atPoint2(L(lm_id))
                lm_data[str(lm_id)] = {'x': float(point[0]), 'y': float(point[1])}
            except Exception:
                pass

        self.diagnostics.compute_landmark_coherence(
            trajectory=traj,
            landmarks=lm_data,
            min_landmark_clearance_m=self.min_landmark_clearance_m,
        )
        stats = self.diagnostics.summary()
        stats['candidate_manager'] = self.candidate_manager.stats()
        write_trajectory_json(path, traj, lm_data, stats=stats)
        self.get_logger().info(
            f'Trayectoria guardada: {len(traj)} poses, '
            f'{len(lm_data)} landmarks → {path}'
        )

    def _log_diagnostics_summary(self):
        summary = self.diagnostics.summary()
        self.get_logger().info(
            'Resumen SLAM: '
            f'keyframes={summary["keyframes"]} '
            f'cand={summary["candidate_landmarks"]} '
            f'seeded={summary["seeded_landmarks"]} '
            f'loops={summary["loop_closures"]} '
            f'gate_ok={summary["gating_acceptances"]} '
            f'gate_rej={summary["gating_rejections"]} '
            f'max_maha={summary["max_maha_sq"]:.2f}'
        )

    @staticmethod
    def _stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @classmethod
    def _timed_pose_from_odom(cls, msg):
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return TimedPose2(
            stamp=cls._stamp_to_seconds(msg.header.stamp),
            x=pose.position.x,
            y=pose.position.y,
            theta=yaw,
        )


def main(args=None):
    rclpy.init(args=args)
    node = GraphSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_trajectory()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
