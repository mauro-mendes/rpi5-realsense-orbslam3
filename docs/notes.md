# Step-by-step: ORB-SLAM3 + RealSense D435i on Raspberry Pi 5

> This is the development log. Each step is added as it is tested and confirmed working.
> Failed approaches are documented alongside successful ones.

## Hardware & environment

- Raspberry Pi 5 (8GB) — **spare RPi5**, used for development and testing
- Intel RealSense D435i (RGB-D + IMU)
- Debian GNU/Linux 12 (Bookworm), aarch64
- Docker 29.3.0
- pyrealsense2 2.58.2 (built from source, venv at `~/realsense-env`)

> **Strategy**: develop and validate entirely on the spare RPi5.
> Once working, migrate to the vest RPi5.
> Future goal: combine ORB-SLAM3 + RealSense + Hailo detection on the vest.

---

## Architecture

```
RPi5 HOST (outside Docker)
  realsense_producer.py  →  ZMQ PUB :5571  →  [Docker: slam_bridge]
  slam_adapter.py        ←  ZMQ SUB :5570  ←  [Docker: slam_bridge]

Docker container (arm64v8/ubuntu:22.04):
  /slam_bridge            — C++ binary: ZMQ + ORB-SLAM3 RGBD (headless)
  /D435i_RGBD.yaml        — camera settings for ORB-SLAM3
  /ORB_SLAM3/             — built from eshan-sud/ORB_SLAM3

No ROS, no GPU, no display — ZMQ only.
```

**Why C++ bridge instead of Python:**
ORB-SLAM3 has no official Python bindings. A C++ ZMQ node calling
the ORB-SLAM3 C++ API directly is more reliable than subprocess hacks.

---

## Step 1 — Prerequisites

Complete the [rpi5-realsense](https://github.com/mauro-mendes/rpi5-realsense)
setup first. You need:
- D435i detected on USB 3.0
- `~/realsense-env` venv with pyrealsense2 2.58.2
- Docker installed

Verify:
```bash
source ~/realsense-env/bin/activate
python setup/verify.py    # from rpi5-realsense repo
docker --version          # expected: 29.x
```

---

## Step 2 — Clone this repo on the spare RPi5

```bash
git clone https://github.com/mauro-mendes/rpi5-realsense-orbslam3.git ~/rpi5-realsense-orbslam3
cd ~/rpi5-realsense-orbslam3
```

---

## Step 3 — Build the Docker image

This downloads and compiles ORB-SLAM3 inside the container.
**Expected build time: 45–90 min on RPi5 (4 cores).**

```bash
cd docker
docker build -t orb-slam3-rpi5 .
```

What happens during build:
1. Install system deps (cmake, opencv, eigen3, libzmq3-dev, etc.)
2. Build Pangolin (headless, no display)
3. Clone + build eshan-sud/ORB_SLAM3
4. Compile `slam_bridge.cpp` (C++ ZMQ + ORB-SLAM3 node)
5. Copy `D435i_RGBD.yaml` (camera settings)

> Leave it running. Monitor with `docker build ... 2>&1 | tee build.log`
> if you want to save the output.

---

## Step 4 — Run

Open **two terminals** on the RPi5.

**Terminal 1 — start the host frame producer:**
```bash
source ~/realsense-env/bin/activate
cd ~/rpi5-realsense-orbslam3
python host/realsense_producer.py
```

Expected output:
```
realsense_producer: publishing on :5571
realsense_producer: depth scale = 0.001000 m/unit
realsense_producer: 30 frames sent
realsense_producer: 60 frames sent
...
```

**Terminal 2 — start the ORB-SLAM3 Docker container:**

Headless (sem display) — salva trajetória em `~/slam_output/`:
```bash
mkdir -p ~/slam_output
docker run --rm --network host \
    -v ~/slam_output:/output \
    orb-slam3-rpi5
```

Com viewer Pangolin (monitor conectado ao RPi5):
```bash
mkdir -p ~/slam_output
xhost +local:docker
docker run --rm --network host \
    -e DISPLAY=:0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device /dev/dri:/dev/dri \
    -v ~/slam_output:/output \
    orb-slam3-rpi5 --viewer
```

> **IMPORTANTE**: `--device /dev/dri:/dev/dri` é obrigatório para passar o GPU do RPi5
> para dentro do container. Sem esse flag, o Pangolin usa software rendering (LLVMpipe)
> e trava o RPi5 por OOM. Com o flag, usa hardware rendering e abre instantaneamente.

Via SSH com X11 forwarding (conectar com `ssh -X user@rpi5`):
```bash
mkdir -p ~/slam_output
docker run --rm --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/slam_output:/output \
    orb-slam3-rpi5 --viewer
```

Ao parar com `Ctrl+C`, o arquivo `~/slam_output/KeyFrameTrajectory.txt` fica salvo no RPi5.
Formato TUM: `timestamp tx ty tz qx qy qz qw` — pode ser plotado com `evo` ou com o `plot_trial.py`.

---

## Step 5b — Gravar o vídeo da sessão

**Vídeo RGB da câmera** (recomendado para experimentos):
```bash
source ~/realsense-env/bin/activate
python host/realsense_producer.py --record --label teste1
# salva em: output/realsense_teste1_YYYYMMDD_HHMMSS.mp4
```

**Screen capture da janela Pangolin** (opcional, para apresentação/paper):

Requer `ffmpeg` instalado (`sudo apt-get install -y ffmpeg`).
Abrir um **terceiro terminal** enquanto o viewer está rodando:

```bash
# Descubra o tamanho da janela do Pangolin primeiro (ou use 1280x720)
ffmpeg -f x11grab -r 30 -s 1280x720 -i :0 \
    ~/slam_output/pangolin_screen.mp4
# Ctrl+C para parar quando terminar o experimento
```

Para capturar só a janela do Pangolin (sem o resto da tela), use `xwininfo` para descobrir a posição:
```bash
xwininfo -name "ORB-SLAM3"
# anote x_offset e y_offset e ajuste: -i :0+x_offset,y_offset
```

**Resumo do que é salvo por sessão:**

| Arquivo | Onde | Como |
|---------|------|------|
| `output/realsense_*.mp4` | host | `--record` no producer |
| `slam_output/KeyFrameTrajectory.txt` | host (via volume) | automático ao parar |
| `slam_output/pangolin_screen.mp4` | host | ffmpeg screen capture (opcional) |

Expected output:
```
slam_bridge: loading ORB-SLAM3...
slam_bridge: ORB-SLAM3 ready
slam_bridge: listening on :5571, publishing on :5570
slam_bridge: 30 frames | OK | pos=(0.00, 0.00, 0.00) yaw=0.00
slam_bridge: 60 frames | OK | pos=(0.12, 0.00, 0.03) yaw=0.01
...
```

The state changes from `LOST` to `OK` after ORB-SLAM3 initializes (usually 1–3 seconds of movement).

---

## Step 5 — Read pose in your application

```python
from host.slam_adapter import SLAMAdapter

slam = SLAMAdapter()
pose = slam.read()   # SLAMPose(x, y, z, yaw, timestamp) or None

if pose and pose.state == "OK":
    print(f"Position: ({pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f}) m  yaw={pose.yaw:.2f} rad")
```

---

## Step 6 — Camera calibration (optional, improves accuracy)

The default `D435i_RGBD.yaml` uses nominal D435i intrinsics (fx=fy=615, cx=320, cy=240).
For better tracking accuracy, replace with real calibration:

```python
import pyrealsense2 as rs
pipeline = rs.pipeline()
profile = pipeline.start()
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
print(f"fx={intr.fx:.2f} fy={intr.fy:.2f} cx={intr.ppx:.2f} cy={intr.ppy:.2f}")
print(f"distortion: {intr.coeffs}")
pipeline.stop()
```

Then update `docker/D435i_RGBD.yaml` and rebuild the image:
```bash
docker build -t orb-slam3-rpi5 .
```

---

## Known risks

| Risk | Severity | Mitigation |
|------|----------|------------|
| Smooth cardboard walls → no features → tracking loss | HIGH | Add colored tape, printed patterns, or posters on walls |
| ORB-SLAM3 build time | LOW | ~60–90 min, done once, image reused |
| D435i USB bandwidth in Docker | MEDIUM | Use `--network host` (not port mapping) |
| Tracking not initialized (LOST state) | MEDIUM | Move camera slowly; ensure textured environment |

---

## References

- eshan-sud/ORB_SLAM3 (RPi5 fork): https://github.com/eshan-sud/ORB_SLAM3
- UZ-SLAMLab/ORB_SLAM3 (original): https://github.com/UZ-SLAMLab/ORB_SLAM3
- Intel RealSense D435i: RGB-D + IMU, 640×480 @ 30fps
- stevenlovegrove/Pangolin: visualization lib (built headless)

---

## Setup log

### 2026-06-22
- RealSense D435i detected: ID 8086:0b3a on Bus 004 (USB 3.0) ✓
- Docker 29.3.0 installed on spare RPi5 ✓
- pyrealsense2 2.58.2 in ~/realsense-env ✓
- Repos created on GitHub ✓
- Wrote slam_bridge.cpp (C++ ZMQ + ORB-SLAM3), CMakeLists.txt, D435i_RGBD.yaml ✓
- Updated Dockerfile to compile slam_bridge ✓

### Docker build — errors encountered and fixes

**Error 1 — Pangolin: `Could NOT find epoxy`**
```
CMake Error: Could NOT find epoxy (missing: epoxy_LIBRARIES epoxy_INCLUDE_DIRS)
```
Fix: add `libepoxy-dev` to apt-get install.

**Error 2 — Pangolin: `Could NOT find OpenGL (missing: OPENGL_opengl_LIBRARY)`**
```
CMake Error: Could NOT find OpenGL (missing: OPENGL_opengl_LIBRARY)
```
Ubuntu 22.04 requires explicit GLVND packages. Fix: add to apt-get:
```
libepoxy-dev libopengl-dev libgl1-mesa-dev libegl1-mesa-dev
libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
```

**Pangolin build: SUCCESS** (124s) ✓

**Error 3 — ORB-SLAM3: `CMake 3.25 or higher is required`**
```
CMake Error: CMake 3.25 or higher is required. You are running version 3.22.1
```
Ubuntu 22.04 ships CMake 3.22. The eshan-sud fork requires 3.25+.
Fix: install CMake from Kitware's official apt repo (added as a Dockerfile layer
**after** Pangolin to preserve that build cache):
```dockerfile
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null \
    | gpg --dearmor - \
    | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' \
    | tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt-get update && apt-get install -y --only-upgrade cmake && \
    cmake --version
```

**Error 4 — ORB-SLAM3 build: RPi5 froze (OOM) at 66%**
```
#10 112.9 [ 66%] Linking CXX executable test_geometry
#10 113.1 [ 66%] Built target test_geometry
client_loop: send disconnect: Connection reset
```
System clock froze at 23:36, SSH dropped, monitor unresponsive — classic OOM during heavy C++ compilation.
Root cause: `./build.sh` calls `make -j$(nproc)` = 4 parallel jobs. Compiling Sophus tests
simultaneously on 4 cores exhausts RAM on RPi5 (even 8GB).

Fix:
- Replace `./build.sh` with manual build steps, using `-j2` throughout
- Disable Sophus tests: `-DBUILD_SOPHUS_TESTS=OFF` (not needed at runtime)
- Hard reboot RPi5 (unplug power), then rebuild

Docker cache note: layers 1–4 (apt, Pangolin, CMake upgrade) are still cached on disk
and will be reused. Only the ORB-SLAM3 layer (step 5) needs to recompile.

**Error 5 — DBoW2: `boost/serialization/serialization.hpp: No such file or directory`**
```
/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/BowVector.h:17:10: fatal error:
    boost/serialization/serialization.hpp: No such file or directory
```
The eshan-sud DBoW2 uses `boost::serialization` (the original DBoW2 doesn't).
Fix: add `libboost-serialization-dev` as a **separate** RUN layer between the CMake
upgrade and the ORB-SLAM3 build, so Pangolin + CMake cache layers are preserved.

**ORB-SLAM3 build: SUCCESS** (1094s ≈ 18 min with -j2) ✓

**Error 6 — slam_bridge: `DBoW2/BowVector.h: No such file or directory`**
```
/ORB_SLAM3/include/KeyFrame.h:24:10: fatal error: DBoW2/BowVector.h: No such file or directory
```
CMakeLists.txt for slam_bridge was missing the DBoW2 include path.
Fix: add `${ORB_SLAM3_DIR}/Thirdparty/DBoW2` to `include_directories`.
Only CMakeLists.txt changes → ORB-SLAM3 layer (step 7) stays cached.

**Error 7 — slam_bridge: `g2o/types/types_six_dof_expmap.h: No such file or directory`**
```
/ORB_SLAM3/include/Converter.h:26:9: fatal error: g2o/types/types_six_dof_expmap.h: No such file or directory
```
Same pattern — g2o include path also missing.
Fix: add ALL Thirdparty paths at once to avoid further iterations:
- include: DBoW2, g2o (added to include_directories)
- link: DBoW2/lib, g2o/lib (added to link_directories + target_link_libraries + rpath)

**Error 8 — slam_bridge: Pangolin gl.hpp — OpenGL functions not declared**
```
/usr/local/include/pangolin/gl/gl.hpp:350:5: error: 'glCopyImageSubDataNV' was not declared
/usr/local/include/pangolin/gl/gl.hpp:524:9: error: 'glDeleteRenderbuffersEXT' was not declared
... (dozens of GL function errors)
```
Root cause: `System.h` → `Tracking.h` → `Viewer.h` → Pangolin → `gl.hpp`.
Pangolin's gl.hpp uses OpenGL extension functions that need GLEW headers to be
included FIRST (before any Pangolin include) to declare them.
Fix:
- Add `#include <GL/glew.h>` as first include in slam_bridge.cpp
- Add `find_package(GLEW REQUIRED)` + `find_package(OpenGL REQUIRED)` to CMakeLists.txt
- Add GLEW/OpenGL to include_directories and target_link_libraries
Note: at RUNTIME in headless mode (`use_viewer=false`), the Pangolin/GL code path
is never executed — only the declarations are needed to compile.

**Error 10 — GLEW vs Epoxy conflict**
```
/usr/include/epoxy/gl.h:38:2: error: #error epoxy/gl.h must be included before (or in place of) GL/gl.h
```
GLEW includes `GL/gl.h` internally. Pangolin (built with epoxy, not GLEW) then tries
to include `epoxy/gl.h` which conflicts. Fix:
- Replace `#include <GL/glew.h>` with `#include <epoxy/gl.h>` in slam_bridge.cpp
- Remove `find_package(GLEW)` and `${GLEW_*}` from CMakeLists.txt
- Add `epoxy` to target_link_libraries

**Error 9 — slam_bridge link: `undefined reference to pangolin::HandlerScroll`**
```
undefined reference to symbol '_ZTVN8pangolin13HandlerScrollE'
/usr/local/lib/libpango_display.so.0: error adding symbols: DSO missing from command line
```
Compilation succeeded (100%) but link failed — Pangolin libraries not in link step.
Fix: add `find_package(Pangolin REQUIRED)` + `${Pangolin_LIBRARIES}` + `${Pangolin_INCLUDE_DIRS}`
to CMakeLists.txt. Pangolin was installed to /usr/local/ via `make install` in the
Pangolin build step, so find_package locates it automatically.

**Docker image build: SUCCESS** ✓
```
[100%] Built target slam_bridge   (14.2s)
COPY D435i_RGBD.yaml              (0.1s)
exporting to image                (179.7s)
naming to docker.io/library/orb-slam3-rpi5:latest  done
```
Total errors encountered and fixed during build: 10
Image ready to run. Proceed to Step 4 (Run).

### 2026-06-23 — End-to-end test: SUCCESS ✓

Full pipeline validated on spare RPi5:

```
RealSense D435i
  → realsense_producer.py (host, ZMQ PUB :5571)
  → slam_bridge (Docker, ORB-SLAM3 RGBD)
  → slam_adapter.py (host, ZMQ SUB :5570)
  → application reads SLAMPose(x, y, z, yaw, state)
```

Sample output from `slam_adapter.py` while moving camera slowly:
```
pos=(0.002, -0.011, -0.002) yaw=0.001 [OK]
pos=(-0.001, -0.000, -0.012) yaw=-0.047 [OK]
pos=(-0.003, 0.000, -0.012) yaw=-0.073 [OK]
```

**Tracking notes:**
- State goes `OK` immediately after map init (~1s with textured surface in view)
- "Fail to track local map!" appears when camera moves too fast → slow down
- Smooth/white walls have few features → add posters or tape for experiments
- Map init message: `New Map created with 946 points` (indoor desktop scene)

**Bug fixed during test:** `SLAMPose` was missing `state` field — added to `host/slam_adapter.py`.

**Viewer test: SUCCESS** ✓
Pangolin opened with Map Viewer (3D point cloud) + Current Frame (live grayscale feed
with ORB feature squares). Status bar: `Maps: 1, KFs: 13, MPs: 607, Matches: 244`.
Fix required: `--device /dev/dri:/dev/dri` to pass RPi5 GPU into Docker container.
Without it: system freezes (OOM from LLVMpipe software rendering).

**Lag fix:** ORB-SLAM3 can't process 30fps in real-time on RPi5 — ZMQ buffer backlog
causes viewer lag. Fix: `--slam-fps 15` in `realsense_producer.py` sends every 2nd frame.

**Trajectory save fix:** `SaveKeyFrameTrajectoryTUM` was after `while(true)` — unreachable.
Added SIGINT/SIGTERM signal handler (`g_running` flag). Now Ctrl+C breaks the loop cleanly
and saves `~/slam_output/KeyFrameTrajectory.txt` before exit.

After Ctrl+C, X11 Pangolin windows stay open as orphans. Close with: `xkill` (click each window).
