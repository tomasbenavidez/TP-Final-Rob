from tp_a_slam_aruco.slam_contracts import (
    REQUIRED_BAG_TOPICS,
    missing_required_topics,
    topic_names_from_rosbag_metadata_text,
)


def test_topic_names_from_rosbag_metadata_text_extracts_topic_metadata_names():
    text = """
    topics_with_message_count:
      - topic_metadata:
          name: /tb4_0/odom
          type: nav_msgs/msg/Odometry
      - topic_metadata:
          name: /tb4_0/scan
          type: sensor_msgs/msg/LaserScan
    """

    assert topic_names_from_rosbag_metadata_text(text) == {
        '/tb4_0/odom',
        '/tb4_0/scan',
    }


def test_missing_required_topics_reports_only_absent_contract_topics():
    present = REQUIRED_BAG_TOPICS - {'/tb4_0/scan', '/tb4_0/tf_static'}

    assert missing_required_topics(present) == ['/tb4_0/scan', '/tb4_0/tf_static']
