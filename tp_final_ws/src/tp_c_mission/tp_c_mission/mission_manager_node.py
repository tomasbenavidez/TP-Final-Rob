#!/usr/bin/env python3
"""Supervisor de exploración informativa y aproximación al primer cono."""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from tp_b_navigation.planner_core import GridPlannerCore
from tp_b_navigation.dynamic_obstacles import apply_dynamic_obstacles
from tp_b_navigation.landmark_io import load_landmark_map
from tp_b_navigation.utils import quaternion_from_yaw
from tp_c_mission.information_exploration import (
    CandidateAction, CoverageBelief, InformationPolicy,
    expected_landmark_fraction, map_signature, observed_free_points,
    sample_candidate_poses, select_approach_pose, select_frontier_action,
)


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        defaults = {
            'robot_radius': 0.18, 'clearance_weight': 0.6, 'clearance_max': 0.5,
            'candidate_spacing': 0.50, 'candidate_yaws': 8,
            'camera_fov': 1.05, 'camera_range': 3.0,
            'landmark_range': 4.0, 'approach_standoff': 0.55,
            'mission_timeout': 300.0, 'coverage_target': 0.95,
            'min_utility': 0.02, 'auto_start': False,
            'landmark_map_file': '',
            'vision_timeout': 3.0,
            'min_goal_new_cells': 8,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def get(name):
            return self.get_parameter(name).value

        self.robot_radius = float(get('robot_radius'))
        self.clearance_weight = float(get('clearance_weight'))
        self.clearance_max = float(get('clearance_max'))
        self.spacing = float(get('candidate_spacing'))
        self.yaw_samples = int(get('candidate_yaws'))
        self.fov = float(get('camera_fov'))
        self.camera_range = float(get('camera_range'))
        self.landmark_range = float(get('landmark_range'))
        self.standoff = float(get('approach_standoff'))
        self.timeout = float(get('mission_timeout'))
        self.coverage_target = float(get('coverage_target'))
        self.min_utility = float(get('min_utility'))
        self.auto_start = bool(get('auto_start'))
        self.vision_timeout = float(get('vision_timeout'))
        self.min_goal_new_cells = int(get('min_goal_new_cells'))

        self.policy = InformationPolicy()
        self.planner = None
        self.coverage = None
        # Planner de navegación: estático + capa dinámica, consistente con el
        # global_planner. Se reconstruye barato (sin recalcular visibilidad) sólo
        # para validar aproximaciones; la exploración sigue sobre el estático.
        self.static_data = None
        self.dynamic_data = None
        self.nav_planner = None
        self.nav_dirty = True
        self.map_signature = None
        self.pose = None
        self.covariance_scale = 1.0
        landmark_map_file = str(get('landmark_map_file'))
        self.landmarks = (
            list(load_landmark_map(landmark_map_file).values())
            if landmark_map_file else [])
        self.candidate_poses = []
        self.visited = set()
        self.exhausted = set()
        self.active = False
        self.approaching = False
        self.current_goal = None
        self.goal_observed_before = 0
        self.pending_goal_evaluation = None
        self.started_at = None
        self.last_vision_at = None
        self.status = 'IDLE'
        self.last_frontier_debug = []
        self.last_selected_action = None

        latched = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, latched)
        self.create_subscription(OccupancyGrid, '/dynamic_obstacles',
                                 self.dynamic_cb, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/mcl_pose', self.pose_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/red_cone_pose',
                                 self.cone_cb, 10)
        self.create_subscription(String, '/navigation_result', self.result_cb, 10)
        self.create_subscription(Bool, '/red_cone/vision_ready', self.vision_cb, 10)
        self.create_subscription(MarkerArray, '/landmarks_markers', self.landmarks_cb,
                                 latched)
        self.goal_pub = self.create_publisher(PoseStamped, '/mission_goal', 10)
        self.cancel_pub = self.create_publisher(Empty, '/mission_cancel', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', latched)
        self.coverage_pub = self.create_publisher(
            MarkerArray, '/mission/coverage_markers', latched)
        self.frontier_pub = self.create_publisher(
            MarkerArray, '/mission/frontier_markers', latched)
        self.create_service(Trigger, '/mission/start', self.start_cb)
        self.create_service(Trigger, '/mission/cancel', self.cancel_cb)
        self.create_timer(1.0, self.loop)
        self._set_status('IDLE')

    def map_cb(self, msg):
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        signature = map_signature(
            data, msg.info.resolution, msg.info.origin.position.x,
            msg.info.origin.position.y)
        if signature == self.map_signature:
            return
        had_map = self.map_signature is not None
        if self.map_signature is not None and self.active:
            self.get_logger().warn(
                'El mapa cambió durante la misión; reinicio planner y cobertura.')
        self.planner = GridPlannerCore.from_occupancy(
            data, msg.info.resolution, msg.info.origin.position.x,
            msg.info.origin.position.y, robot_radius=self.robot_radius,
            clearance_weight=self.clearance_weight, clearance_max=self.clearance_max,
            allow_unknown=False)
        self.static_data = data
        self.nav_planner = None
        self.nav_dirty = True
        self.coverage = CoverageBelief(self.planner)
        self.map_signature = signature
        self.candidate_poses = list(sample_candidate_poses(
            self.planner, self.spacing, self.yaw_samples))
        # El ray-casting de cada vista candidata es invariante al estado del
        # robot: se precalcula una vez por mapa en vez de en cada tick.
        self.coverage.precompute_visibility(
            self.candidate_poses, self.fov, self.camera_range)
        if had_map:
            self.visited.clear()
            self.exhausted.clear()
            self.current_goal = None
            self.pending_goal_evaluation = None

    def dynamic_cb(self, msg):
        if self.planner is None:
            return
        if (msg.info.width != self.planner.width
                or msg.info.height != self.planner.height):
            return
        self.dynamic_data = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        self.nav_dirty = True

    def _navigation_planner(self):
        """Grilla estático + dinámica, igual que el global_planner.

        Sin capa dinámica disponible, cae al planner estático (compatibilidad
        con el robot real o con el monitor apagado).
        """
        if self.planner is None or self.static_data is None or self.dynamic_data is None:
            return self.planner
        if self.nav_planner is None or self.nav_dirty:
            merged = apply_dynamic_obstacles(self.static_data, self.dynamic_data)
            self.nav_planner = GridPlannerCore.from_occupancy(
                merged, self.planner.resolution, self.planner.origin_x,
                self.planner.origin_y, robot_radius=self.robot_radius,
                clearance_weight=self.clearance_weight,
                clearance_max=self.clearance_max, allow_unknown=False)
            self.nav_dirty = False
        return self.nav_planner

    def pose_cb(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (pose.position.x, pose.position.y, yaw)
        cov = msg.pose.covariance
        planar_trace = max(0.0, cov[0] + cov[7] + cov[35])
        self.covariance_scale = max(0.5, min(3.0, planar_trace / 0.10))

    def landmarks_cb(self, msg):
        if msg.markers:
            self.landmarks = [(m.pose.position.x, m.pose.position.y) for m in msg.markers]

    def vision_cb(self, msg):
        if msg.data:
            self.last_vision_at = self.get_clock().now()

    def start_cb(self, _request, response):
        if self.planner is None or self.pose is None or not self._vision_is_fresh():
            response.success = False
            response.message = 'Faltan /map, /mcl_pose o cámara/TF disponible.'
            return response
        self.active = True
        self.approaching = False
        self.current_goal = None
        self.pending_goal_evaluation = None
        self.last_frontier_debug = []
        self.last_selected_action = None
        self.started_at = self.get_clock().now()
        self._set_status('EXPLORING')
        response.success = True
        response.message = 'Misión iniciada.'
        return response

    def cancel_cb(self, _request, response):
        self.cancel_pub.publish(Empty())
        self.active = False
        self.current_goal = None
        self.approaching = False
        self.last_selected_action = None
        self._set_status('IDLE')
        response.success = True
        response.message = 'Misión cancelada.'
        return response

    def cone_cb(self, msg):
        if not self.active or self.approaching or self.planner is None or self.pose is None:
            return
        cone = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # Validar contra el mismo grid que planea el navegador (estático +
        # obstáculos dinámicos), para no proponer poses que A* después rechaza.
        approach = select_approach_pose(
            self._navigation_planner(), self.pose[:2], cone,
            standoff=self.standoff, samples=24)
        if approach is None:
            self.get_logger().warn('Cono confirmado sin aproximación alcanzable; sigo explorando.')
            return
        pose, _path = approach
        self.approaching = True
        self._set_status('APPROACHING')
        self._publish_goal(pose)

    def result_cb(self, msg):
        result = msg.data
        if result == 'PREEMPTED' and self.approaching:
            return
        if result == 'REACHED' and self.approaching:
            self.active = False
            self.current_goal = None
            self.last_selected_action = None
            self._set_status('FOUND')
        elif result == 'REACHED':
            if self.current_goal is not None:
                cell = self.planner.world_to_cell(*self.current_goal[:2])
                self.pending_goal_evaluation = (cell, self.goal_observed_before)
            self.current_goal = None
        elif result in ('PLAN_FAILED', 'TIMEOUT'):
            if self.current_goal is not None:
                cell = self.planner.world_to_cell(*self.current_goal[:2])
                self.visited.add(cell)
                self.exhausted.add(cell)
            self.current_goal = None
            self.last_selected_action = None
            if self.approaching:
                self.approaching = False
                self._set_status('EXPLORING')

    def loop(self):
        ready_to_autostart = (
            self.auto_start and not self.active and self.status == 'IDLE'
            and self.planner is not None and self.pose is not None)
        if ready_to_autostart:
            request, response = Trigger.Request(), Trigger.Response()
            self.start_cb(request, response)
        if not self.active or self.planner is None or self.pose is None:
            return
        if not self._vision_is_fresh():
            self.cancel_pub.publish(Empty())
            self.active = False
            self.current_goal = None
            self.last_selected_action = None
            self._set_status('FAILED')
            self.get_logger().error('Misión detenida: cámara o TF sin datos recientes.')
            return
        if self.started_at is not None:
            elapsed = (self.get_clock().now() - self.started_at).nanoseconds * 1e-9
            if elapsed >= self.timeout:
                self._finish_not_found('timeout')
                return
        if self.approaching:
            return
        self.coverage.observe(self.pose, self.fov, self.camera_range)
        self._evaluate_reached_goal()
        self._publish_coverage_markers()
        if self.current_goal is not None:
            return
        if self.coverage.coverage_fraction() >= self.coverage_target:
            self._finish_not_found('cobertura agotada')
            return
        action = self._select_action()
        self._publish_frontier_markers(action)
        if action is None or self.policy.utility(action, self.covariance_scale) < self.min_utility:
            self._finish_not_found('sin acciones informativas')
            return
        self._publish_goal(action.pose)

    def _select_action(self):
        if not self.candidate_poses:
            return None
        # Primero rankear por información barata; calcular A* sólo para las mejores vistas.
        ranked = []
        max_visible_cells = max(
            1.0,
            self.fov * self.camera_range ** 2 /
            (2.0 * self.planner.resolution ** 2),
        )
        for pose in self.candidate_poses:
            unseen = self.coverage.unseen_count(pose, self.fov, self.camera_range)
            if unseen == 0:
                continue
            coverage_gain = min(1.0, unseen / max_visible_cells)
            localization_gain = expected_landmark_fraction(
                pose, self.landmarks, self.planner, self.fov, self.landmark_range)
            ranked.append((coverage_gain + 0.35 * localization_gain, pose,
                           coverage_gain, localization_gain))
        ranked.sort(reverse=True, key=lambda item: item[0])
        diagonal = math.hypot(self.planner.width, self.planner.height) * self.planner.resolution
        candidates = []
        for _, pose, coverage_gain, localization_gain in ranked:
            cell = self.planner.world_to_cell(*pose[:2])
            if cell in self.exhausted:
                continue
            path = self.planner.plan_world(self.pose[:2], pose[:2], simplify=False)
            if path is None:
                continue
            path_cost = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                            for a, b in zip(path, path[1:])) / max(diagonal, 1e-6)
            risk = min(1.0, float(self.planner.cost[cell]) / max(self.clearance_max, 1e-6))
            repetition = 6.0 if cell in self.visited else 0.0
            candidates.append(CandidateAction(
                pose, coverage_gain, localization_gain, path_cost, risk, repetition))
            if len(candidates) >= 40:
                break
        selected = self.policy.select(candidates, self.covariance_scale)
        selected_cell = (
            None if selected is None else self.planner.world_to_cell(*selected.pose[:2]))
        if selected is not None and selected_cell not in self.visited and self.policy.utility(selected, self.covariance_scale) >= self.min_utility:
            self.last_frontier_debug = []
            self.last_selected_action = selected
            return selected
        frontier, debug = select_frontier_action(
            self.planner, self.coverage, self.pose[:2], self.candidate_poses,
            self.fov, self.camera_range, exhausted_cells=self.exhausted)
        self.last_frontier_debug = debug
        self.last_selected_action = frontier if frontier is not None else selected
        return frontier if frontier is not None else selected

    def _publish_goal(self, pose):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(pose[0]), float(pose[1])
        msg.pose.orientation = quaternion_from_yaw(float(pose[2]))
        self.goal_pub.publish(msg)
        self.current_goal = pose
        self.goal_observed_before = self._observed_free_count()
        self.visited.add(self.planner.world_to_cell(*pose[:2]))

    def _finish_not_found(self, reason):
        self.cancel_pub.publish(Empty())
        self.active = False
        self.current_goal = None
        self.last_selected_action = None
        self._set_status('NOT_FOUND')
        self.get_logger().warn(f'Misión terminada sin cono: {reason}.')

    def _set_status(self, status):
        self.status = status
        self.status_pub.publish(String(data=status))

    def _vision_is_fresh(self):
        if self.last_vision_at is None:
            return False
        age = (self.get_clock().now() - self.last_vision_at).nanoseconds * 1e-9
        return age <= self.vision_timeout

    def _observed_free_count(self):
        if self.coverage is None or self.planner is None:
            return 0
        free = ~self.planner.blocked
        return int((self.coverage.observed & free).sum())

    def _evaluate_reached_goal(self):
        if self.pending_goal_evaluation is None:
            return
        cell, observed_before = self.pending_goal_evaluation
        gained = self._observed_free_count() - int(observed_before)
        if gained < self.min_goal_new_cells:
            self.exhausted.add(cell)
        self.pending_goal_evaluation = None

    def _publish_coverage_markers(self):
        if self.coverage is None or self.planner is None:
            return
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'mission_coverage'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.planner.resolution
        marker.scale.y = self.planner.resolution
        marker.scale.z = 0.02
        marker.pose.orientation.w = 1.0
        marker.color = ColorRGBA(r=0.1, g=0.45, b=1.0, a=0.28)
        for x, y in observed_free_points(self.planner, self.coverage):
            marker.points.append(Point(x=float(x), y=float(y), z=0.02))
        self.coverage_pub.publish(MarkerArray(markers=[marker]))

    def _publish_frontier_markers(self, selected):
        markers = []
        delete = Marker()
        delete.header.frame_id = 'map'
        delete.header.stamp = self.get_clock().now().to_msg()
        delete.action = Marker.DELETEALL
        markers.append(delete)

        for index, (_score, action, _frontier_seen, _unseen) in enumerate(self.last_frontier_debug[:12]):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = delete.header.stamp
            marker.ns = 'mission_frontiers'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(action.pose[0])
            marker.pose.position.y = float(action.pose[1])
            marker.pose.position.z = 0.08
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.12
            marker.color = ColorRGBA(r=1.0, g=0.58, b=0.08, a=0.85)
            markers.append(marker)

        selected = selected or self.last_selected_action
        if selected is not None:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = delete.header.stamp
            marker.ns = 'mission_selected_goal'
            marker.id = 100
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = float(selected.pose[0])
            marker.pose.position.y = float(selected.pose[1])
            marker.pose.position.z = 0.10
            marker.pose.orientation = quaternion_from_yaw(float(selected.pose[2]))
            marker.scale.x = 0.35
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color = ColorRGBA(r=0.1, g=1.0, b=0.35, a=0.95)
            markers.append(marker)
        self.frontier_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
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
