from tp_a_slam_aruco.slam_contracts import (
    REQUIRED_BAG_TOPICS,
    missing_required_topics,
    required_bag_topics,
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


def test_required_bag_topics_follow_selected_namespace():
    assert required_bag_topics('tb4_1') == {
        '/tb4_1/odom',
        '/tb4_1/scan',
        '/tb4_1/oakd/rgb/preview/image_raw',
        '/tb4_1/oakd/rgb/preview/camera_info',
        '/tb4_1/tf',
        '/tb4_1/tf_static',
    }


def test_missing_required_topics_accepts_selected_namespace():
    present = required_bag_topics('tb4_1') - {'/tb4_1/scan'}

    assert missing_required_topics(present, robot_namespace='tb4_1') == [
        '/tb4_1/scan'
    ]
