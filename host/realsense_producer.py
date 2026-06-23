"""
RealSense producer — runs on the RPi5 host (outside Docker).

Reads RGB + Depth from RealSense D435i and publishes frames
to the ORB-SLAM3 Docker container via ZMQ (:5571).

Usage:
    source ~/realsense-env/bin/activate
    python host/realsense_producer.py
    python host/realsense_producer.py --record          # saves RGB video
    python host/realsense_producer.py --record --label test1
    python host/realsense_producer.py --slam-fps 15    # throttle SLAM to 15fps
"""
import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import zmq

parser = argparse.ArgumentParser()
parser.add_argument("--record",   action="store_true", help="Save RGB video to output/")
parser.add_argument("--label",    default="",          help="Label appended to filename")
parser.add_argument("--slam-fps", type=int, default=30,
                    help="Max FPS sent to SLAM container (default 30). "
                         "Use 10-15 on RPi5 to avoid lag.")
args = parser.parse_args()

WIDTH    = 640
HEIGHT   = 480
CAM_FPS  = 30

# How many camera frames to skip between SLAM publishes
# e.g. slam_fps=15 → send every 2nd frame; slam_fps=10 → every 3rd
slam_skip = max(1, round(CAM_FPS / args.slam_fps))

# ZMQ publisher
ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.setsockopt(zmq.SNDHWM, 2)   # drop frames if consumer can't keep up
pub.bind("tcp://127.0.0.1:5571")
print(f"realsense_producer: publishing on :5571  (slam-fps={args.slam_fps}, skip={slam_skip})")

# RealSense pipeline
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.rgb8, CAM_FPS)
cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16,  CAM_FPS)
align = rs.align(rs.stream.color)

profile = pipeline.start(cfg)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
print(f"realsense_producer: depth scale = {depth_scale:.6f} m/unit")

# Video writer (optional)
video_writer = None
if args.record:
    out_dir = Path("output")
    out_dir.mkdir(exist_ok=True)
    ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
    lbl = f"_{args.label}" if args.label else ""
    vid_path = out_dir / f"realsense{lbl}_{ts}.mp4"
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_writer = cv2.VideoWriter(str(vid_path), fourcc, CAM_FPS, (WIDTH, HEIGHT))
    print(f"realsense_producer: recording to {vid_path}")

frame_count  = 0
slam_count   = 0
try:
    while True:
        frames = align.process(pipeline.wait_for_frames())
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color = np.asanyarray(color_frame.get_data())   # uint8, HxWx3, RGB
        depth = np.asanyarray(depth_frame.get_data())   # uint16, HxW

        # Throttle frames sent to SLAM
        if frame_count % slam_skip == 0:
            header = json.dumps({
                "width":       WIDTH,
                "height":      HEIGHT,
                "timestamp":   time.monotonic(),
                "frame":       slam_count,
                "depth_scale": depth_scale,
            }).encode()
            pub.send_multipart([header, color.tobytes(), depth.tobytes()], zmq.NOBLOCK)
            slam_count += 1

        if video_writer is not None:
            video_writer.write(cv2.cvtColor(color, cv2.COLOR_RGB2BGR))

        frame_count += 1
        if frame_count % 30 == 0:
            print(f"realsense_producer: {frame_count} cam frames | {slam_count} slam frames sent")

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    if video_writer is not None:
        video_writer.release()
        print(f"realsense_producer: video saved.")
    pipeline.stop()
    pub.close()
    ctx.term()
