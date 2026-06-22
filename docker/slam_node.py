"""
ORB-SLAM3 node — runs inside Docker container.

Receives RGB + Depth frames from host via ZMQ (:5571),
feeds ORB-SLAM3 frame by frame, publishes pose (x, y, z, yaw)
back to host via ZMQ (:5570).

Host architecture:
    realsense_producer.py  →  ZMQ :5571  →  [this node]
    [this node]            →  ZMQ :5570  →  SLAMAdapter (VestController)
"""
import json
import struct
import numpy as np
import zmq

# ZMQ setup (network host — sees 127.0.0.1 of the RPi5 host)
ctx = zmq.Context()

sub = ctx.socket(zmq.SUB)
sub.connect("tcp://127.0.0.1:5571")
sub.setsockopt_string(zmq.SUBSCRIBE, "")

pub = ctx.socket(zmq.PUB)
pub.bind("tcp://127.0.0.1:5570")

print("slam_node: waiting for frames on :5571 ...")

# TODO: initialize ORB-SLAM3 Python bindings or subprocess wrapper
# Reference: /ORB_SLAM3/Examples/RGB-D/rgbd_realsense_D435i

frame_count = 0

while True:
    parts = sub.recv_multipart()
    # Expected: [header_json, rgb_bytes, depth_bytes]
    if len(parts) != 3:
        continue

    header = json.loads(parts[0])
    rgb = np.frombuffer(parts[1], dtype=np.uint8).reshape(
        header["height"], header["width"], 3
    )
    depth = np.frombuffer(parts[2], dtype=np.uint16).reshape(
        header["height"], header["width"]
    )

    frame_count += 1

    # --- Feed ORB-SLAM3 (placeholder) ---
    # pose_matrix = slam.TrackRGBD(rgb, depth, header["timestamp"])
    # x, y, z, yaw = extract_pose(pose_matrix)

    x, y, z, yaw = 0.0, 0.0, 0.0, 0.0  # placeholder until ORB-SLAM3 is wired

    pose = {
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
        "timestamp": header["timestamp"],
        "frame": frame_count,
    }
    pub.send_string(json.dumps(pose))

    if frame_count % 30 == 0:
        print(f"slam_node: {frame_count} frames processed | pose=({x:.2f}, {y:.2f}, {z:.2f}) yaw={yaw:.2f}")
