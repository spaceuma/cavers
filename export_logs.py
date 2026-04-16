# MIT License
#
# Copyright (c) 2026 Space Robotics Lab at UMA
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Extracts mcap rosbags.

This script scans the mcap bags and extracts the messages and timestamps.
"""

__author__ = "Levin Gerdes and Hugo Leblond and Giacomo Franchini"


import argparse
import csv
import os
from enum import Enum, auto
from typing import Any, Callable, Dict, List, TextIO, Tuple

import cv2 as cv2
import imageio
import numpy as np
import progressbar  # type: ignore
import open3d as o3d
import open3d.core as o3c
from rosbags.interfaces import Connection
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


indices_dict = {
    "THERMAL": 0,
    "RS_DEPTH": 0,
    "RS_COLOR": 0,
    "VELODYNE_CLOUD": 0,
}

device = o3c.Device("CPU:0")

ROS_DTYPE_MAP = {
    1: np.int8,    # INT8
    2: np.uint8,   # UINT8
    3: np.int16,   # INT16
    4: np.uint16,  # UINT16
    5: np.int32,   # INT32
    6: np.uint32,  # UINT32
    7: np.float32, # FLOAT32
    8: np.float64  # FLOAT64
}

class Sensors(Enum):
    THERMAL = auto()
    RS_DEPTH = auto()
    RS_COLOR = auto()
    IMU = auto()
    VELODYNE_CLOUD = auto()
    GT_ODOM = auto()
    TF = auto()
    TF_STATIC = auto()


def get_args() -> argparse.Namespace:
    """Parses CLI arguments"""

    parser = argparse.ArgumentParser(description="")

    # fmt: off
    parser.add_argument("--input-path",  "-i",    type=str,   default="~/cavers/dataset",   dest="input_path",    help="Parent directory of input mcaps. E.g. '/path/to/recordings' with subdirs 'loc_diablo_1' etc.")
    parser.add_argument("--output-path", "-o",    type=str,   default="~/cavers/extracted", dest="output_path",   help="Output will be written here")
    parser.add_argument("--compression", "-c",    type=int,   default=9,                               dest="png_comp_rate", help="PNG compression [0..9], lowest to highest rate")
    parser.add_argument("--precision",   "-p",    type=int,   default=12,                              dest="precision",     help="Num. of digits behind decimal point in CSV")
    # fmt: on

    return parser.parse_args()


def format_floats(input: List[Any], precision: int = 12) -> List[Any]:
    """
    Use positional instead of scientific notation for all floats in input array.
    Values that are not floats are left unchanged.
    """
    return [
        (
            np.format_float_positional(x, precision, trim="-")
            if isinstance(x, float)
            else x
        )
        for x in input
    ]


def export_rgb_img(msg, timestamp, path: str, writer: csv.DictWriter, keys: List[str]) -> None:
    # Save RGB image as PNG
    img_name = f"RS_COLOR_{indices_dict['RS_COLOR']}.png"
    img = np.frombuffer(msg.data, dtype=np.uint8)
    img = img.reshape((msg.height, msg.width, -1))
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imwrite(os.path.join(path, "data", img_name), img, [cv2.IMWRITE_PNG_COMPRESSION, args.png_comp_rate])

    # Write corresponding line to CSV
    val = [timestamp, img_name]
    writer.writerow(dict(zip(keys, val)))

    indices_dict['RS_COLOR'] += 1

def export_rs_depth(msg, timestamp, path: str, writer: csv.DictWriter, keys: List[str]) -> None:
    # Save depth image as PNG
    img_name = f"RS_DEPTH_{indices_dict['RS_DEPTH']}.png"
    depth = np.frombuffer(msg.data, dtype=np.uint16)
    depth = np.minimum(np.maximum(depth, 0), 65535)  # limit to [0,max16bit]
    depth = np.reshape(depth, (msg.height, msg.width))
    imageio.imsave(os.path.join(path, "data", img_name), depth)

    # Write corresponding line to CSV
    val = [timestamp, img_name]
    writer.writerow(dict(zip(keys, val)))
    indices_dict['RS_DEPTH'] += 1


def export_thermal_img(msg, timestamp, path: str, writer: csv.DictWriter, keys: List[str]) -> None:
    # Save thermal image as PNG
    img_name = f"THERMAL_{indices_dict['THERMAL']}.png"
    img = np.frombuffer(msg.data, dtype=np.uint16)
    img = np.minimum(np.maximum(img, 0), 65535)  # limit to [0,max16bit]
    img = np.reshape(img, (msg.height, msg.width))
    imageio.imsave(os.path.join(path, "data", img_name), img)

    # Write corresponding line to CSV
    val = [timestamp, img_name]
    writer.writerow(dict(zip(keys, val)))
    indices_dict['THERMAL'] += 1


def export_imu(msg, timestamp, _, writer: csv.DictWriter, keys: List[str]) -> None:
    """
    Read and export IMU messages.

    The IMU does not provide covariances (i.e., all covariance matrices only contain 0s),
    so we do not export them.

    Messages are of type IMU:
    https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html
    """
    val = [
        timestamp,
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z,
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z,
    ]

    writer.writerow(dict(zip(keys, format_floats(val, args.precision))))

def export_tf(msg, _0, _1, writer: csv.DictWriter, keys: List[str]) -> None:
    # https://docs.ros2.org/foxy/api/tf2_msgs/msg/TFMessage.html

    for t in msg.transforms:
        val = [
            # Timestamps are part of the individual stamped transforms,
            # not of the overall tf message.
            int(str(t.header.stamp.sec) + str(t.header.stamp.nanosec).zfill(9)),
            t.header.frame_id,
            t.child_frame_id,
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ]

        writer.writerow(dict(zip(keys, format_floats(val, args.precision))))


def export_odom(msg, timestamp, _, writer: csv.DictWriter, keys: List[str]) -> None:
    """
    Read and export Odometry messages.
    
    Messages are of type nav_msgs::msg::Odometry:
    https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html
    """
    val = [
        timestamp,
        msg.header.frame_id,
        msg.child_frame_id,
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z,
        msg.twist.twist.angular.x,
        msg.twist.twist.angular.y,
        msg.twist.twist.angular.z,
    ]

    writer.writerow(dict(zip(keys, format_floats(val, args.precision))))


def export_point_cloud(msg, timestamp, path: str, writer: csv.DictWriter, keys: List[str]) -> None:
    """
    Export PointCloud2 messages to PCD format.
    
    Messages are of type sensor_msgs::msg::PointCloud2:
    https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html
    """
    # Save PointCloud2 message to PCD format
    pcd_name = f"VELODYNE_CLOUD_{indices_dict['VELODYNE_CLOUD']}.pcd"
    field_names = ("x", "y", "z", "intensity", "ring", "time")
    
    names = [f.name for f in msg.fields]
    formats = [ROS_DTYPE_MAP[f.datatype] for f in msg.fields]
    offsets = [f.offset for f in msg.fields]

    structured_dtype = np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': msg.point_step
    })

    points = np.frombuffer(msg.data, dtype=structured_dtype)
    if len(points) == 0:
        print("Empty points!")
        return
    try:
        xyz = np.column_stack((points['x'], points['y'], points['z'])).astype(np.float32)
        intensity = points['intensity'].astype(np.float32).reshape(-1, 1)
        ring = points['ring'].astype(np.uint16).reshape(-1, 1)
        time = points['time'].astype(np.float64).reshape(-1, 1)
    except ValueError as e:
        print(f"Frame {indices_dict['VELODYNE_CLOUD']}: Missing expected Velodyne fields. Skipping... ({e})")
        return

    pcd = o3d.t.geometry.PointCloud(device)
    pcd.point.positions = o3c.Tensor(xyz)
    pcd.point.intensity = o3c.Tensor(intensity)
    pcd.point.ring = o3c.Tensor(ring)
    pcd.point.time = o3c.Tensor(time)
    o3d.t.io.write_point_cloud(os.path.join(path, 'data', pcd_name), pcd)
    
    # Write corresponding line to CSV
    val = [timestamp, pcd_name]
    writer.writerow(dict(zip(keys, val)))
    indices_dict['VELODYNE_CLOUD'] += 1


def get_message_and_timestamp(connection: Connection, rawdata: bytes) -> tuple[Any, int | None]:
    """Returns the deserialized message and its timestamp"""
    msg = deserialize_cdr(rawdata, connection.msgtype)
    timestamp = None
    if hasattr(msg, "header"):
        timestamp = int(
            str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec).zfill(9)
        )
    return msg, timestamp


if __name__ == "__main__":
    args = get_args()

    INPUT_PATH: str = os.path.expanduser(args.input_path)
    OUTPUT_PATH: str = os.path.expanduser(args.output_path)
    print(f"Input path: {INPUT_PATH}\nOutput path: {OUTPUT_PATH}")

    # Get names of subdirectories in INPUT_PATH
    # E.g. "loc_diablo_1"
    SUBDIRS: List[str] = [
        directory
        for directory in os.listdir(INPUT_PATH)
        if os.path.isdir(os.path.join(INPUT_PATH, directory))
    ]

    # Map topic names to Sensor enum and function that can read and export it
    topic_map: Dict[str, Tuple[Sensors, Callable]] = {
        "/spaceuma/realsense2_camera_node/depth/image_rect_raw": (Sensors.RS_DEPTH, export_rs_depth),
        "/spaceuma/realsense2_camera_node/color/image_raw": (Sensors.RS_COLOR, export_rgb_img),
        "/spaceuma/thermal_camera_node/normalized": (Sensors.THERMAL, export_thermal_img),
        "/spaceuma/realsense2_camera_node/imu": (Sensors.IMU, export_imu),
        "/spaceuma/optitrack/odom": (Sensors.GT_ODOM, export_odom),
        "/spaceuma/velodyne_points": (Sensors.VELODYNE_CLOUD, export_point_cloud),
        "/tf": (Sensors.TF, export_tf),
        "/tf_static": (Sensors.TF_STATIC, export_tf),
    }

    # Map from sensor enum to header entries / column names for the corresponding CSV
    header_map: Dict[Sensors, List[str]] = {}

    header_map[Sensors.IMU] = [
        "Timestamp",
        "Angular_Velocity_X",
        "Angular_Velocity_Y",
        "Angular_Velocity_Z",
        "Linear_Acceleration_X",
        "Linear_Acceleration_Y",
        "Linear_Acceleration_Z",
    ]
    header_map[Sensors.GT_ODOM] = [
        "Timestamp",
        "Frame_ID",
        "Child_Frame_ID",
        "PX",
        "PY",
        "PZ",
        "QX",
        "QY",
        "QZ",
        "QW",
        "VX",
        "VY",
        "VZ",
        "VROLL",
        "VPITCH",
        "VYAW",
    ]
    header_map[Sensors.TF] = [
        "Timestamp",
        "Frame_ID",
        "Child_Frame_ID",
        "TX",
        "TY",
        "TZ",
        "QX",
        "QY",
        "QZ",
        "QW",
    ]
    header_map[Sensors.TF_STATIC] = header_map[Sensors.TF]
    header_map[Sensors.RS_DEPTH] = ["Timestamp", "Image_Name"]
    header_map[Sensors.RS_COLOR] = ["Timestamp", "Image_Name"]
    header_map[Sensors.THERMAL] = ["Timestamp", "Image_Name"]
    header_map[Sensors.VELODYNE_CLOUD] = ["Timestamp", "PCD_Name"]

    for directory in SUBDIRS:
        # Setup export directories
        abs_out_dir: str = os.path.join(OUTPUT_PATH, directory)
        for s in Sensors:
            if s in [
                Sensors.RS_COLOR,
                Sensors.RS_DEPTH,
                Sensors.THERMAL,
                Sensors.VELODYNE_CLOUD
            ]:
                os.makedirs(name=os.path.join(abs_out_dir, s.name, "data"), exist_ok=True)
            else:
                os.makedirs(name=os.path.join(abs_out_dir, s.name), exist_ok=True)

        # Setup CSV Writers with paths to current export directory
        csvs: Dict[Sensors, TextIO] = {}
        csv_writers: Dict[Sensors, csv.DictWriter] = {}
        for s in Sensors:
            csvs[s] = open(os.path.join(abs_out_dir, s.name, f"data.csv"), "w")
            csv_writers[s] = csv.DictWriter(
                csvs[s],
                fieldnames=header_map[s],
                dialect="unix",
                quoting=csv.QUOTE_NONE,
            )
            csv_writers[s].writeheader()

        # Read and export mcap in current directory
        with Reader(os.path.join(INPUT_PATH, directory)) as reader:
            print(f"Extracting bag: {directory}")

            num_messages = sum(c.msgcount for c in reader.connections)

            bar = progressbar.ProgressBar(maxval=num_messages)
            bar.start()
            progress = 0
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic in topic_map.keys():
                    s: Sensors = topic_map[connection.topic][0]
                    f: Callable = topic_map[connection.topic][1]
                    msg, time = get_message_and_timestamp(connection, rawdata)
                    out_path = os.path.join(abs_out_dir, s.name) # Sensor directory path
                    f(msg, time, out_path, csv_writers[s], header_map[s])
                progress += 1
                bar.update(progress)

        for c in csvs.values():
            c.close()
        
        for key in indices_dict.keys():
            indices_dict[key] = 0
