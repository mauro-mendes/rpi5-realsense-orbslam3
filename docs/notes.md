# Notes — rpi5-realsense-orbslam3

## Hardware

- Raspberry Pi 5 (8GB) — spare unit, used for development
- Intel RealSense D435i
- Debian GNU/Linux 12 (Bookworm), aarch64
- Docker 29.3.0

## Architecture

```
RPi5 HOST
  realsense_producer.py  →  ZMQ :5571  →  Docker (slam_node.py)
  Docker (slam_node.py)  →  ZMQ :5570  →  SLAMAdapter

Docker container:
  Base: arm64v8/ubuntu:22.04
  ORB-SLAM3: github.com/eshan-sud/ORB_SLAM3 (RPi5 fork)
  No GPU, no ROS2 — pure ZMQ communication
```

## Setup log

### 2026-06-22

- RealSense D435i detected: `ID 8086:0b3a` on Bus 004 (USB 3.0)
- Docker 29.3.0 already installed on spare RPi5
- pyrealsense2 venv created: `~/realsense-env`
- Repos created: rpi5-realsense + rpi5-realsense-orbslam3

## References

- eshan-sud/ORB_SLAM3 (RPi5 fork): https://github.com/eshan-sud/ORB_SLAM3
- Paper: "An In-depth Evaluation of ORB-SLAM3 on the Raspberry Pi 5"
- Intel RealSense D435i datasheet: RGB-D + Stereo-Inertial, 30fps @ 640x480
- Issue — RealSense on RPi5 without ROS2: https://github.com/IntelRealSense/realsense-ros/issues/3065

## Known risks

| Risk | Severity | Mitigation |
|------|----------|------------|
| Smooth cardboard walls → tracking loss | HIGH | Add colored tape or printed patterns |
| ORB-SLAM3 build time | LOW | ~30–60 min, done once, image reused |
| Monocular Pi Camera — no metric scale | N/A | Using D435i RGB-D — metric scale available |
