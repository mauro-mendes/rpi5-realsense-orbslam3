"""
RealSense producer — runs on the RPi5 host (outside Docker).

Reads RGB + Depth + IMU from RealSense D435i and publishes:
  :5571 — video frames (header_json + rgb_bytes + depth_bytes)
  :5572 — IMU samples  (json: {type, x, y, z, t})

Uses callback API (required for reliable IMU + video acquisition).
Video and IMU timestamps come from the RealSense hardware clock (ms → s)
so they are directly comparable in the slam_bridge for IMU interpolation.

Usage:
    source ~/realsense-env/bin/activate
    python host/realsense_producer.py
    python host/realsense_producer.py --record          # saves RGB video
    python host/realsense_producer.py --record --label test1
    python host/realsense_producer.py --slam-fps 10    # throttle SLAM to 10fps
"""
import argparse
import json
import queue
import threading
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
                         "Use 10 on RPi5 — IMU fills the gaps between frames.")
args = parser.parse_args()

WIDTH   = 640
HEIGHT  = 480
CAM_FPS = 30

# Skip N camera frames between each SLAM publish
# e.g. slam-fps=10 → send every 3rd frame; slam-fps=15 → every 2nd
slam_skip = max(1, round(CAM_FPS / args.slam_fps))

# ------------------------------------------------------------------
# ZMQ sockets (one per thread to respect ZMQ's thread-safety rules)
# ------------------------------------------------------------------
ctx = zmq.Context()

# Video publisher (main thread only)
pub = ctx.socket(zmq.PUB)
pub.setsockopt(zmq.SNDHWM, 2)   # drop frames if SLAM consumer lags
pub.bind("tcp://127.0.0.1:5571")

# IMU publisher (imu_thread only)
imu_pub = ctx.socket(zmq.PUB)
imu_pub.setsockopt(zmq.SNDHWM, 400)  # ~1s of IMU at 400Hz
imu_pub.bind("tcp://127.0.0.1:5572")

print(f"realsense_producer: video on :5571, IMU on :5572  "
      f"(slam-fps={args.slam_fps}, skip={slam_skip})")

# ------------------------------------------------------------------
# Thread-safe queues between SDK callback and processing threads
# ------------------------------------------------------------------
video_q: queue.SimpleQueue = queue.SimpleQueue()   # video framesets
imu_q: queue.SimpleQueue   = queue.SimpleQueue()   # (type, x, y, z, t) tuples

# ------------------------------------------------------------------
# RealSense pipeline — callback API (required for IMU + video)
# ------------------------------------------------------------------
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.rgb8, CAM_FPS)
cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16,  CAM_FPS)
cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, 400)

align = rs.align(rs.stream.color)


def on_frame(frame: rs.frame) -> None:
    """SDK callback — called from an internal SDK thread.
    Only puts frames into queues; no ZMQ calls here (not thread-safe)."""
    if frame.is_frameset():
        video_q.put(frame)
    elif frame.is_motion_frame():
        mf = frame.as_motion_frame()
        d  = mf.get_motion_data()
        t  = mf.get_timestamp() / 1000.0  # hardware ms → seconds
        stream_type = "gyro" if mf.profile.stream_type() == rs.stream.gyro else "accel"
        imu_q.put((stream_type, d.x, d.y, d.z, t))


profile  = pipeline.start(cfg, on_frame)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
print(f"realsense_producer: depth_scale = {depth_scale:.6f} m/unit")

# ------------------------------------------------------------------
# IMU publisher thread (only thread that touches imu_pub socket)
# ------------------------------------------------------------------
stop_event = threading.Event()


def imu_pub_loop() -> None:
    while not stop_event.is_set():
        try:
            stream_type, x, y, z, t = imu_q.get(timeout=0.1)
        except queue.Empty:
            continue
        msg = json.dumps({"type": stream_type,
                          "x": float(x), "y": float(y), "z": float(z),
                          "t": t}).encode()
        try:
            imu_pub.send(msg, zmq.NOBLOCK)
        except zmq.Again:
            pass  # consumer (slam_bridge) not ready yet — drop


imu_thread = threading.Thread(target=imu_pub_loop, daemon=True)
imu_thread.start()

# ------------------------------------------------------------------
# Video writer (optional --record)
# ------------------------------------------------------------------
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

# ------------------------------------------------------------------
# Main loop — processes video frames from queue
# ------------------------------------------------------------------
frame_count = 0
slam_count  = 0

try:
    while True:
        # Block until a video frameset is available (timeout to allow Ctrl+C)
        try:
            raw_fs = video_q.get(timeout=2.0)
        except queue.Empty:
            continue

        aligned     = align.process(raw_fs)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color = np.asanyarray(color_frame.get_data())   # uint8 HxWx3 RGB
        depth = np.asanyarray(depth_frame.get_data())   # uint16 HxW

        # Throttle frames sent to SLAM (every slam_skip-th frame)
        if frame_count % slam_skip == 0:
            # Use hardware timestamp so it matches IMU timestamps in slam_bridge
            hw_ts = color_frame.get_timestamp() / 1000.0  # ms → seconds
            header = json.dumps({
                "width":       WIDTH,
                "height":      HEIGHT,
                "timestamp":   hw_ts,
                "frame":       slam_count,
                "depth_scale": depth_scale,
            }).encode()
            try:
                pub.send_multipart([header, color.tobytes(), depth.tobytes()],
                                   zmq.NOBLOCK)
            except zmq.Again:
                pass  # SLAM is behind — frame dropped (SNDHWM=2 handles backpressure)
            slam_count += 1

        if video_writer is not None:
            video_writer.write(cv2.cvtColor(color, cv2.COLOR_RGB2BGR))

        frame_count += 1
        if frame_count % 90 == 0:
            print(f"realsense_producer: {frame_count} cam frames | "
                  f"{slam_count} slam sent | imu_q={imu_q.qsize()}")

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    stop_event.set()
    imu_thread.join(timeout=1.0)
    if video_writer is not None:
        video_writer.release()
        print("realsense_producer: video saved.")
    pipeline.stop()
    pub.close()
    imu_pub.close()
    ctx.term()
