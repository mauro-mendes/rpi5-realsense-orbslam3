FROM arm64v8/ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Base dependencies
RUN apt-get update && apt-get install -y \
    build-essential cmake git wget curl gnupg lsb-release \
    libopencv-dev libopencv-contrib-dev \
    libeigen3-dev libglew-dev \
    libssl-dev libusb-1.0-0-dev \
    libzmq3-dev \
    libepoxy-dev libopengl-dev libgl1-mesa-dev libegl1-mesa-dev \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Python deps for host scripts (producer / adapter run on host, but include here for dev)
RUN pip3 install pyzmq numpy

# Pangolin (ORB-SLAM3 visualization dependency — built headless, no display needed)
RUN git clone --depth 1 https://github.com/stevenlovegrove/Pangolin /Pangolin && \
    mkdir /Pangolin/build && cd /Pangolin/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) && make install && ldconfig && \
    rm -rf /Pangolin

# CMake 3.25+ required by eshan-sud ORB-SLAM3 (Ubuntu 22.04 ships 3.22)
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null \
    | gpg --dearmor - \
    | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' \
    | tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt-get update && apt-get install -y --only-upgrade cmake && \
    rm -rf /var/lib/apt/lists/* && \
    cmake --version

# ORB-SLAM3 (eshan-sud fork, tested on aarch64 Debian/Ubuntu)
RUN git clone https://github.com/eshan-sud/ORB_SLAM3 /ORB_SLAM3 && \
    cd /ORB_SLAM3 && chmod +x build.sh && ./build.sh

# slam_bridge: C++ ZMQ + ORB-SLAM3 node
COPY CMakeLists.txt  /bridge_src/CMakeLists.txt
COPY slam_bridge.cpp /bridge_src/slam_bridge.cpp
RUN mkdir /bridge_src/build && cd /bridge_src/build && \
    cmake .. && make -j$(nproc) && \
    cp /bridge_src/build/slam_bridge /slam_bridge

# Camera settings
COPY D435i_RGBD.yaml /D435i_RGBD.yaml

# Default: headless (no display)
# With viewer: docker run ... -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix orb-slam3-rpi5 --viewer
CMD ["/slam_bridge"]
