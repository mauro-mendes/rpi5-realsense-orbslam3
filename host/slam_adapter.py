"""
SLAMAdapter — receives pose from ORB-SLAM3 Docker container via ZMQ.

Interface analogous to ToFSensor: call read() to get the latest SLAMPose or None.
Runs a background thread to drain the ZMQ socket continuously.
"""
import json
import threading
import time
from dataclasses import dataclass
from typing import Optional

import zmq


@dataclass
class SLAMPose:
    x: float         # meters, forward
    y: float         # meters, lateral
    z: float         # meters, vertical
    yaw: float       # radians
    timestamp: float


class SLAMAdapter:
    def __init__(self, host: str = "127.0.0.1", port: int = 5570):
        self._latest: Optional[SLAMPose] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()

        ctx = zmq.Context()
        self._sock = ctx.socket(zmq.SUB)
        self._sock.connect(f"tcp://{host}:{port}")
        self._sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self._sock.setsockopt(zmq.RCVTIMEO, 100)  # ms

        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def read(self) -> Optional[SLAMPose]:
        with self._lock:
            return self._latest

    def stop(self) -> None:
        self._stop.set()

    def _recv_loop(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self._sock.recv_string()
                data = json.loads(msg)
                pose = SLAMPose(
                    x=data["x"],
                    y=data["y"],
                    z=data["z"],
                    yaw=data["yaw"],
                    timestamp=data["timestamp"],
                )
                with self._lock:
                    self._latest = pose
            except zmq.Again:
                pass  # timeout — no data yet
