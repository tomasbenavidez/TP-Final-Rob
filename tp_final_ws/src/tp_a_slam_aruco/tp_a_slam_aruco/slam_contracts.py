import re

from tp_platform.platform_profiles import tb4_topics


def required_bag_topics(robot_namespace='tb4_0'):
    topics = tb4_topics(robot_namespace)
    return {
        topics.odom_topic,
        topics.scan_topic,
        topics.rgb_topic,
        topics.camera_info_topic,
        topics.tf_topic,
        topics.tf_static_topic,
    }


REQUIRED_BAG_TOPICS = required_bag_topics('tb4_0')


def topic_names_from_rosbag_metadata_text(text):
    """Extract topic names from a rosbag2 metadata.yaml text."""
    return set(re.findall(r'^\s*name:\s*(/\S+)\s*$', text, flags=re.MULTILINE))


def missing_required_topics(present_topics, required_topics=None, robot_namespace='tb4_0'):
    if required_topics is None:
        required_topics = required_bag_topics(robot_namespace)
    return sorted(required_topics - set(present_topics))
