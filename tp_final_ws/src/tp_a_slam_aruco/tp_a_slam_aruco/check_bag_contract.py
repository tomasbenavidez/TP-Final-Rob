#!/usr/bin/env python3
import argparse
from pathlib import Path

from tp_a_slam_aruco.slam_contracts import (
    missing_required_topics,
    required_bag_topics,
    topic_names_from_rosbag_metadata_text,
)


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Verify that a rosbag2 metadata.yaml contains Parte A topics.'
    )
    parser.add_argument(
        'metadata',
        help='Path to a rosbag2 metadata.yaml file or to the bag directory.',
    )
    parser.add_argument(
        '--robot-namespace',
        default='tb4_0',
        help='Namespace del TurtleBot4 esperado: tb4_0 o tb4_1.',
    )
    parsed = parser.parse_args(args=args)

    path = Path(parsed.metadata)
    if path.is_dir():
        path = path / 'metadata.yaml'

    text = path.read_text()
    topics = topic_names_from_rosbag_metadata_text(text)
    required = required_bag_topics(parsed.robot_namespace)
    missing = missing_required_topics(topics, required_topics=required)

    if missing:
        print(f'Missing required Parte A bag topics for {parsed.robot_namespace}:')
        for topic in missing:
            print(f'  - {topic}')
        raise SystemExit(1)

    print(f'Parte A bag contract OK for {parsed.robot_namespace}:')
    for topic in sorted(required):
        print(f'  - {topic}')


if __name__ == '__main__':
    main()
