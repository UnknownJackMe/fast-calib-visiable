#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
import rosbag2_py
import yaml
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


SUPPORTED_TYPES = {
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
}


def storage_id_from_metadata(bag_path):
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        return "sqlite3"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    return metadata["rosbag2_bagfile_information"].get("storage_identifier", "sqlite3")


def open_reader(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_path),
            storage_id=storage_id_from_metadata(bag_path),
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def topic_types(reader):
    return {item.name: item.type for item in reader.get_all_topics_and_types()}


def image_timestamps(bag_path, image_topic):
    reader = open_reader(bag_path)
    types = topic_types(reader)
    if image_topic not in types:
        available = [f"{name} [{msg_type}]" for name, msg_type in sorted(types.items())]
        raise RuntimeError(
            f"Image topic {image_topic!r} is not in the bag. Available topics:\n  "
            + "\n  ".join(available)
        )
    if types[image_topic] not in SUPPORTED_TYPES:
        raise RuntimeError(
            f"Unsupported image type {types[image_topic]!r}; expected sensor_msgs/msg/Image "
            "or sensor_msgs/msg/CompressedImage"
        )

    timestamps = []
    while reader.has_next():
        topic, _data, timestamp = reader.read_next()
        if topic == image_topic:
            timestamps.append(timestamp)
    if not timestamps:
        raise RuntimeError(f"No messages found on image topic {image_topic!r}")
    return types[image_topic], timestamps


def extract_at_timestamp(bag_path, image_topic, image_type, target_timestamp):
    reader = open_reader(bag_path)
    message_class = get_message(image_type)
    bridge = CvBridge()
    closest = None

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != image_topic:
            continue
        distance = abs(timestamp - target_timestamp)
        if closest is None or distance < closest[0]:
            closest = (distance, timestamp, data)

    if closest is None:
        raise RuntimeError(f"No messages found on image topic {image_topic!r}")

    _distance, timestamp, data = closest
    message = deserialize_message(data, message_class)
    if image_type == "sensor_msgs/msg/CompressedImage":
        image = bridge.compressed_imgmsg_to_cv2(message, desired_encoding="bgr8")
        encoding = message.format
    else:
        image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        encoding = message.encoding
    return image, timestamp, encoding


def main():
    parser = argparse.ArgumentParser(
        description="Extract the temporal-middle camera frame from a ROS 2 bag"
    )
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--image-topic", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--metadata-output", type=Path)
    args = parser.parse_args()

    bag_path = args.bag.resolve()
    if not (bag_path / "metadata.yaml").exists():
        raise RuntimeError(f"Not a ROS 2 bag directory: {bag_path}")

    image_type, timestamps = image_timestamps(bag_path, args.image_topic)
    target_timestamp = timestamps[len(timestamps) // 2]
    image, timestamp, encoding = extract_at_timestamp(
        bag_path, args.image_topic, image_type, target_timestamp
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), image):
        raise RuntimeError(f"Failed to write extracted image to {args.output}")

    metadata = {
        "bag_path": str(bag_path),
        "image_topic": args.image_topic,
        "image_type": image_type,
        "source_encoding": encoding,
        "selected_timestamp_ns": int(timestamp),
        "image_message_count": len(timestamps),
        "width": int(image.shape[1]),
        "height": int(image.shape[0]),
        "output_path": str(args.output.resolve()),
    }
    if args.metadata_output:
        args.metadata_output.parent.mkdir(parents=True, exist_ok=True)
        args.metadata_output.write_text(
            yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
        )

    print(
        f"Extracted {args.image_topic} [{image_type}] frame {len(timestamps) // 2 + 1}/"
        f"{len(timestamps)} to {args.output} ({image.shape[1]}x{image.shape[0]})"
    )


if __name__ == "__main__":
    main()
