# rpi5-realsense-orbslam3

ORB-SLAM3 in Docker with Intel RealSense D435i on Raspberry Pi 5.

Provides metric 6DOF pose (x, y, z, yaw) in real time using RGB-D SLAM —
no ROS2 required. Communication between host and container via ZMQ.

Tested on **Debian Bookworm (12), aarch64**, Docker 29.3.0.

> **Prerequisite:** complete [rpi5-realsense](https://github.com/mauro-mendes/rpi5-realsense) setup first.

---

## Architecture

```
RPi5 HOST
  realsense_producer.py  ──ZMQ :5571──▶  Docker (slam_node.py)
  slam_adapter.py        ◀─ZMQ :5570──   Docker (slam_node.py)

Docker container:
  Base image : arm64v8/ubuntu:22.04
  ORB-SLAM3  : github.com/eshan-sud/ORB_SLAM3 (RPi5 fork)
  No ROS2    : ZMQ only
```

---

## Setup

### 1. Build the Docker image (first time ~30–60 min)

```bash
cd docker
docker build -t orb-slam3-rpi5 .
```

### 2. Run

Terminal 1 — host producer:
```bash
source ~/realsense-env/bin/activate
python host/realsense_producer.py
```

Terminal 2 — SLAM container:
```bash
docker run --rm --network host orb-slam3-rpi5
```

### 3. Read pose in your application

```python
from host.slam_adapter import SLAMAdapter

slam = SLAMAdapter()
pose = slam.read()   # SLAMPose(x, y, z, yaw, timestamp) or None
```

---

## File structure

```
docker/
  Dockerfile              # arm64v8/ubuntu:22.04 + ORB-SLAM3 + pyzmq
  slam_node.py            # ZMQ subscriber → ORB-SLAM3 → ZMQ publisher
host/
  realsense_producer.py   # reads D435i, publishes RGB+Depth via ZMQ
  slam_adapter.py         # background thread, exposes read() → SLAMPose
docs/
  notes.md                # setup log, references, known issues
```

---

## Notes

See [docs/notes.md](docs/notes.md) for setup log, references and known risks.

---

## Related

- [rpi5-realsense](https://github.com/mauro-mendes/rpi5-realsense) — RealSense D435i setup and examples on RPi5
