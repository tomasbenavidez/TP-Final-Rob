import math

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quaternion_components(theta):
    return math.sin(theta / 2.0), math.cos(theta / 2.0)


def build_belief_message(map_frame, stamp, pose2):
    msg = PoseStamped()
    msg.header.frame_id = map_frame
    msg.header.stamp = stamp
    msg.pose.position.x = pose2.x()
    msg.pose.position.y = pose2.y()
    msg.pose.orientation.z, msg.pose.orientation.w = yaw_to_quaternion_components(
        pose2.theta()
    )
    return msg


def build_pose_array_message(map_frame, stamp, poses):
    msg = PoseArray()
    msg.header.frame_id = map_frame
    msg.header.stamp = stamp
    for pose2 in poses:
        pose = Pose()
        pose.position.x = pose2.x()
        pose.position.y = pose2.y()
        pose.orientation.z, pose.orientation.w = yaw_to_quaternion_components(
            pose2.theta()
        )
        msg.poses.append(pose)
    return msg


def build_path_message(map_frame, stamp, poses):
    msg = Path()
    msg.header.frame_id = map_frame
    msg.header.stamp = stamp
    for pose2 in poses:
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose.position.x = pose2.x()
        pose_stamped.pose.position.y = pose2.y()
        pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w = (
            yaw_to_quaternion_components(pose2.theta())
        )
        msg.poses.append(pose_stamped)
    return msg


def build_landmark_markers(map_frame, stamp, landmarks):
    marker_array = MarkerArray()
    for landmark_id, point in landmarks:
        marker = Marker()
        marker.header.frame_id = map_frame
        marker.header.stamp = stamp
        marker.ns = 'aruco_opt'
        marker.id = landmark_id
        marker.type = Marker.SPHERE
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15
        marker.color.g = 1.0
        marker.color.a = 0.9
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.orientation.w = 1.0
        marker_array.markers.append(marker)
    return marker_array


def build_base_debug_markers(base_frame, stamp, observations):
    marker_array = MarkerArray()
    for observation in observations:
        marker = Marker()
        marker.header.frame_id = base_frame
        marker.header.stamp = stamp
        marker.ns = 'aruco_base_debug'
        marker.id = int(observation['id'])
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.10
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.9
        marker.pose.position.x = float(observation['x_base'])
        marker.pose.position.y = float(observation['y_base'])
        marker.pose.orientation.w = 1.0
        marker_array.markers.append(marker)
    return marker_array


def build_map_to_odom_transform(map_frame, odom_frame, stamp, pose2):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = map_frame
    msg.child_frame_id = odom_frame
    msg.transform.translation.x = pose2.x()
    msg.transform.translation.y = pose2.y()
    msg.transform.translation.z = 0.0
    msg.transform.rotation.z, msg.transform.rotation.w = yaw_to_quaternion_components(
        pose2.theta()
    )
    return msg
