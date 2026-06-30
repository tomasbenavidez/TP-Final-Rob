import re


REQUIRED_BAG_TOPICS = {
    '/tb4_0/odom',
    '/tb4_0/scan',
    '/tb4_0/oakd/rgb/preview/image_raw',
    '/tb4_0/oakd/rgb/preview/camera_info',
    '/tb4_0/tf',
    '/tb4_0/tf_static',
}


def topic_names_from_rosbag_metadata_text(text):
    """Extract topic names from a rosbag2 metadata.yaml text."""
    return set(re.findall(r'^\s*name:\s*(/\S+)\s*$', text, flags=re.MULTILINE))


def missing_required_topics(present_topics, required_topics=REQUIRED_BAG_TOPICS):
    return sorted(required_topics - set(present_topics))
