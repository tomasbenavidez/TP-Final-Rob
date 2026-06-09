#!/usr/bin/env python3

import bisect
import json
import math
import os

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from tp_slam_aruco.slam_geometry import lookup_planar_transform, planar_graph_from_transforms
from tp_slam_aruco.slam_motion_filter import (
    MotionFilterConfig,
    estimate_motion_sample,
    should_integrate_scan,
)

_L_OCC = 0.85
_L_FREE = -0.40
_L_MIN = -5.0
_L_MAX = 5.0


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')

        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width', 300)
        self.declare_parameter('height', 300)
        self.declare_parameter('origin_x', -4.0)
        self.declare_parameter('origin_y', -1.5)
        self.declare_parameter('scan_topic', 'tb4_0/scan')
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('map_output', '/tmp/mapa')
        self.declare_parameter('publish_every', 50)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('scan_frame', 'rplidar_link')
        self.declare_parameter('tf_static_topic', '/tb4_0/tf_static')
        self.declare_parameter('use_tf_static_extrinsics', True)
        self.declare_parameter('lidar_tx', -0.04)
        self.declare_parameter('lidar_ty', 0.0)
        self.declare_parameter('lidar_yaw', math.pi / 2.0)
        self.declare_parameter('enable_motion_filter', True)
        self.declare_parameter('max_angular_speed_rad_s', 0.10)
        self.declare_parameter('max_path_curvature_rad_m', 0.35)
        self.declare_parameter('max_lateral_pose_change_m', 0.03)
        self.declare_parameter('still_linear_speed_m_s', 0.03)
        self.declare_parameter('motion_sample_dt_s', 0.10)

        self.resolution = self.get_parameter('resolution').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.map_output = self.get_parameter('map_output').value
        self.publish_every = self.get_parameter('publish_every').value
        self.base_frame = self.get_parameter('base_frame').value
        self.scan_frame = self.get_parameter('scan_frame').value
        self.tf_static_topic = self.get_parameter('tf_static_topic').value
        self.use_tf_static_extrinsics = self.get_parameter('use_tf_static_extrinsics').value
        self.lidar_tx = float(self.get_parameter('lidar_tx').value)
        self.lidar_ty = float(self.get_parameter('lidar_ty').value)
        self.lidar_yaw = float(self.get_parameter('lidar_yaw').value)
        self.motion_sample_dt_s = float(self.get_parameter('motion_sample_dt_s').value)
        self.motion_filter_config = MotionFilterConfig(
            enable_motion_filter=bool(self.get_parameter('enable_motion_filter').value),
            max_angular_speed_rad_s=float(
                self.get_parameter('max_angular_speed_rad_s').value
            ),
            max_path_curvature_rad_m=float(
                self.get_parameter('max_path_curvature_rad_m').value
            ),
            max_lateral_pose_change_m=float(
                self.get_parameter('max_lateral_pose_change_m').value
            ),
            still_linear_speed_m_s=float(
                self.get_parameter('still_linear_speed_m_s').value
            ),
        )
        self._lidar_tf_resolved = not self.use_tf_static_extrinsics
        self._static_graph = {}

        traj_path = self.get_parameter('trajectory_file').value
        self.trajectory = []
        self.traj_stamps = []
        if traj_path and os.path.exists(traj_path):
            with open(traj_path) as handle:
                data = json.load(handle)
            self.trajectory = data['trajectory']
            self.traj_stamps = [pose['stamp'] for pose in self.trajectory]
            self.get_logger().info(
                f'Trayectoria cargada: {len(self.trajectory)} poses desde {traj_path}'
            )
        else:
            self.get_logger().error(
                f'trajectory_file no encontrado: "{traj_path}". '
                'Ejecutar primero la 1ª pasada (parte_a_slam.launch.py).'
            )

        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.scan_count = 0
        self.received_scan_count = 0
        self.skipped_turn_rate_count = 0
        self.skipped_curvature_count = 0
        self.skipped_lateral_count = 0
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_cb,
            best_effort,
        )

        tf_static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            TFMessage,
            self.tf_static_topic,
            self.tf_static_cb,
            tf_static_qos,
        )

        self.get_logger().info(
            f'occupancy_grid_node activo. '
            f'Grilla {self.width}x{self.height} @ {self.resolution} m/cel '
            f'({self.width * self.resolution:.1f}x{self.height * self.resolution:.1f} m). '
            f'Origen ({self.origin_x}, {self.origin_y}). '
            f'LIDAR fallback tx={self.lidar_tx} ty={self.lidar_ty} yaw={self.lidar_yaw:.3f}. '
            f'motion_filter={self.motion_filter_config.enable_motion_filter}'
        )

    def tf_static_cb(self, msg):
        self._static_graph.update(planar_graph_from_transforms(msg.transforms))
        try:
            planar_tf = lookup_planar_transform(
                graph=self._static_graph,
                source_frame=self.base_frame,
                target_frame=self.scan_frame,
            )
        except KeyError:
            return

        self.lidar_tx = planar_tf.tx
        self.lidar_ty = planar_tf.ty
        self.lidar_yaw = planar_tf.yaw
        if not self._lidar_tf_resolved:
            self.get_logger().info(
                f'Extrínseco LIDAR resuelto desde tf_static: '
                f'tx={self.lidar_tx:.4f} ty={self.lidar_ty:.4f} yaw={self.lidar_yaw:.4f}'
            )
        self._lidar_tf_resolved = True

    def _world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def _in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        x, y = x0, y0
        if dx > dy:
            err = dx // 2
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy // 2
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        cells.append((x1, y1))
        return cells

    def _interpolate_pose(self, t):
        if not self.traj_stamps:
            return None
        if t <= self.traj_stamps[0]:
            pose = self.trajectory[0]
            return pose['x'], pose['y'], pose['theta']
        if t >= self.traj_stamps[-1]:
            pose = self.trajectory[-1]
            return pose['x'], pose['y'], pose['theta']

        hi = bisect.bisect_right(self.traj_stamps, t)
        lo = hi - 1
        t0, t1 = self.traj_stamps[lo], self.traj_stamps[hi]
        alpha = (t - t0) / (t1 - t0) if t1 != t0 else 0.0

        p0, p1 = self.trajectory[lo], self.trajectory[hi]
        x = p0['x'] + alpha * (p1['x'] - p0['x'])
        y = p0['y'] + alpha * (p1['y'] - p0['y'])
        dth = math.atan2(
            math.sin(p1['theta'] - p0['theta']),
            math.cos(p1['theta'] - p0['theta']),
        )
        theta = p0['theta'] + alpha * dth
        return x, y, theta

    def _motion_sample_at(self, stamp):
        dt = self.motion_sample_dt_s
        prev_pose = self._interpolate_pose(stamp - dt)
        curr_pose = self._interpolate_pose(stamp)
        next_pose = self._interpolate_pose(stamp + dt)
        if prev_pose is None or curr_pose is None or next_pose is None:
            return None
        return estimate_motion_sample(prev_pose, curr_pose, next_pose, dt)

    def _should_integrate_scan(self, stamp):
        sample = self._motion_sample_at(stamp)
        if sample is None:
            return True, 'boundary'
        return should_integrate_scan(sample, self.motion_filter_config)

    def _log_scan_filter_summary(self):
        self.get_logger().info(
            'Filtro de scans: '
            f'recibidos={self.received_scan_count} '
            f'integrados={self.scan_count} '
            f'desc_turn_rate={self.skipped_turn_rate_count} '
            f'desc_curvature={self.skipped_curvature_count} '
            f'desc_lateral={self.skipped_lateral_count}'
        )

    def integrate_scan(self, robot_pose, scan):
        rx, ry, rth = robot_pose

        lx = rx + self.lidar_tx * math.cos(rth) - self.lidar_ty * math.sin(rth)
        ly = ry + self.lidar_tx * math.sin(rth) + self.lidar_ty * math.cos(rth)
        lth = rth + self.lidar_yaw

        gx0, gy0 = self._world_to_grid(lx, ly)
        angle = scan.angle_min
        for distance in scan.ranges:
            if not (
                math.isnan(distance)
                or math.isinf(distance)
                or distance < scan.range_min
                or distance > scan.range_max
            ):
                ray_angle = lth + angle
                wx = lx + distance * math.cos(ray_angle)
                wy = ly + distance * math.sin(ray_angle)
                gx1, gy1 = self._world_to_grid(wx, wy)

                cells = self._bresenham(gx0, gy0, gx1, gy1)
                for cx, cy in cells[:-1]:
                    if self._in_bounds(cx, cy):
                        self.log_odds[cy, cx] = max(_L_MIN, self.log_odds[cy, cx] + _L_FREE)

                cx, cy = cells[-1]
                if self._in_bounds(cx, cy):
                    self.log_odds[cy, cx] = min(_L_MAX, self.log_odds[cy, cx] + _L_OCC)

            angle += scan.angle_increment

    def scan_cb(self, msg):
        self.received_scan_count += 1
        if not self.trajectory:
            return
        if self.use_tf_static_extrinsics and not self._lidar_tf_resolved:
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        allowed, reason = self._should_integrate_scan(stamp)
        if not allowed:
            if reason == 'turn_rate':
                self.skipped_turn_rate_count += 1
            elif reason == 'curvature':
                self.skipped_curvature_count += 1
            elif reason == 'lateral':
                self.skipped_lateral_count += 1
            if self.received_scan_count % self.publish_every == 0:
                self._log_scan_filter_summary()
            return

        pose = self._interpolate_pose(stamp)
        if pose is None:
            return

        self.integrate_scan(pose, msg)
        self.scan_count += 1
        if self.received_scan_count % self.publish_every == 0:
            self._log_scan_filter_summary()
            self.publish_map()

    def publish_map(self):
        prob = (1.0 / (1.0 + np.exp(-self.log_odds))).astype(np.float32)

        occ = np.full(self.height * self.width, -1, dtype=np.int8)
        flat = prob.flatten()
        occ[flat >= 0.6] = 100
        occ[flat <= 0.4] = 0

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = float(self.resolution)
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = occ.tolist()
        self.map_pub.publish(msg)

    def export_map(self, path_prefix):
        prob = 1.0 / (1.0 + np.exp(-self.log_odds))

        pgm = np.full((self.height, self.width), 205, dtype=np.uint8)
        pgm[prob >= 0.6] = 0
        pgm[prob <= 0.4] = 254
        pgm = np.flipud(pgm)

        pgm_path = path_prefix + '.pgm'
        os.makedirs(os.path.dirname(pgm_path) if os.path.dirname(pgm_path) else '.', exist_ok=True)
        with open(pgm_path, 'wb') as handle:
            handle.write(f'P5\n{self.width} {self.height}\n255\n'.encode('ascii'))
            handle.write(pgm.tobytes())

        yaml_path = path_prefix + '.yaml'
        with open(yaml_path, 'w') as handle:
            handle.write(f'image: {os.path.basename(pgm_path)}\n')
            handle.write(f'resolution: {self.resolution}\n')
            handle.write(f'origin: [{self.origin_x:.4f}, {self.origin_y:.4f}, 0.0]\n')
            handle.write('negate: 0\n')
            handle.write('occupied_thresh: 0.65\n')
            handle.write('free_thresh: 0.196\n')

        self.get_logger().info(
            f'Mapa exportado → {pgm_path} '
            f'({self.width}x{self.height} px, '
            f'{self.width * self.resolution:.1f}x{self.height * self.resolution:.1f} m). '
            f'Scans recibidos={self.received_scan_count} integrados={self.scan_count}.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_map()
        if node.map_output:
            node.export_map(node.map_output)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
