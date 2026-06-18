#!/usr/bin/env python3
import argparse
from pathlib import Path

from tp_slam_aruco.slam_contracts import (
    missing_required_topics,
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
    parsed = parser.parse_args(args=args)

    path = Path(parsed.metadata)
    if path.is_dir():
        path = path / 'metadata.yaml'

    text = path.read_text()
    topics = topic_names_from_rosbag_metadata_text(text)
    missing = missing_required_topics(topics)

    if missing:
        print('Missing required Parte A bag topics:')
        for topic in missing:
            print(f'  - {topic}')
        raise SystemExit(1)

    print('Parte A bag contract OK:')
    for topic in sorted(topics):
        print(f'  - {topic}')


if __name__ == '__main__':
    main()
