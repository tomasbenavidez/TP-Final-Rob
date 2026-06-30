#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfBridgeNode(Node):
    def __init__(self):
        super().__init__('tf_bridge_node')

        self.declare_parameter('bag_tf_topic', '/tb4_0/tf')
        self.declare_parameter('bag_tf_static_topic', '/tb4_0/tf_static')
        self.declare_parameter('tf_topic', '/tf')
        self.declare_parameter('tf_static_topic', '/tf_static')

        bag_tf_topic = self.get_parameter('bag_tf_topic').value
        bag_tf_static_topic = self.get_parameter('bag_tf_static_topic').value
        tf_topic = self.get_parameter('tf_topic').value
        tf_static_topic = self.get_parameter('tf_static_topic').value

        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        tf_static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )

        self.tf_pub = self.create_publisher(TFMessage, tf_topic, tf_qos)
        self.tf_static_pub = self.create_publisher(TFMessage, tf_static_topic, tf_static_qos)

        self.create_subscription(TFMessage, bag_tf_topic, self._republish_tf, tf_qos)
        self.create_subscription(
            TFMessage,
            bag_tf_static_topic,
            self._republish_tf_static,
            tf_static_qos,
        )

        self.get_logger().info(
            f'tf_bridge_node republishing {bag_tf_topic}->{tf_topic} '
            f'and {bag_tf_static_topic}->{tf_static_topic}'
        )

    def _republish_tf(self, msg):
        self.tf_pub.publish(msg)

    def _republish_tf_static(self, msg):
        self.tf_static_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
