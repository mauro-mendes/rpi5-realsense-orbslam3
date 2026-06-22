"""
RealSense producer — runs on the RPi5 host (outside Docker).

Reads RGB + Depth from RealSense D435i and publishes frames
to the ORB-SLAM3 Docker container via ZMQ (:5571).

Usage:
    source ~/realsense-env/bin/activate
    python host/realsense_producer.py
"""
import json
import time
import numpy as np
import zmq
import pyrealsense2 as rs

WIDTH = 640
HEIGHT = 480
FPS = 30

# ZMQ publisher
ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://127.0.0.1:5571")
print("realsense_producer: publishing on :5571")

# RealSense pipeline
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.rgb8, FPS)
cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
align = rs.align(rs.stream.color)

profile = pipeline.start(cfg)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
print(f"realsense_producer: depth scale = {depth_scale:.6f} m/unit")

frame_count = 0
try:
    while True:
        frames = align.process(pipeline.wait_for_frames())
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color = np.asanyarray(color_frame.get_data())   # uint8, HxWx3, RGB
        depth = np.asanyarray(depth_frame.get_data())   # uint16, HxW

        header = json.dumps({
            "width": WIDTH,
            "height": HEIGHT,
            "timestamp": time.monotonic(),
            "frame": frame_count,
            "depth_scale": depth_scale,
        }).encode()

        pub.send_multipart([header, color.tobytes(), depth.tobytes()])
        frame_count += 1

        if frame_count % 30 == 0:
            print(f"realsense_producer: {frame_count} frames sent")
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    pipeline.stop()
    pub.close()
    ctx.term()
