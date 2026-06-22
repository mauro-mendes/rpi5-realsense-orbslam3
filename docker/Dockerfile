FROM arm64v8/ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Base dependencies
RUN apt-get update && apt-get install -y \
    build-essential cmake git wget curl gnupg lsb-release \
    libopencv-dev libopencv-contrib-dev \
    libeigen3-dev libglew-dev \
    libssl-dev libusb-1.0-0-dev \
    python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

# librealsense2 — Intel official packages for Ubuntu 22.04 ARM64
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
    | tee /etc/apt/trusted.gpg.d/librealsense.pgp > /dev/null && \
    echo "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && apt-get install -y \
    librealsense2-utils librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

# Pangolin (ORB-SLAM3 visualization dependency)
RUN git clone --depth 1 https://github.com/stevenlovegrove/Pangolin /Pangolin && \
    mkdir /Pangolin/build && cd /Pangolin/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && make install && \
    rm -rf /Pangolin

# ORB-SLAM3 — RPi5 fork (eshan-sud, tested on Debian Bookworm)
RUN git clone https://github.com/eshan-sud/ORB_SLAM3 /ORB_SLAM3 && \
    cd /ORB_SLAM3 && chmod +x build.sh && ./build.sh

# Python deps for slam_node.py
RUN pip3 install pyzmq numpy pyrealsense2

COPY slam_node.py /slam_node.py

CMD ["python3", "/slam_node.py"]
