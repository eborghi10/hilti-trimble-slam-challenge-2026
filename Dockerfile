# CUDA 11.8 on Ubuntu 22.04 - GPU runtime available for OpenVINS CUDA feature tracking
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2 Humble
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release software-properties-common \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
       http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-cv-bridge \
    ros-humble-pcl-conversions \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-std-srvs \
    ros-humble-message-filters \
    ros-humble-visualization-msgs \
    ros-humble-image-transport \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Install build tools and system dependencies (apt OpenCV avoids conflict with cv_bridge)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    libeigen3-dev \
    libboost-all-dev \
    libpcl-dev \
    libsuitesparse-dev \
    libopencv-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libcudnn8 \
    libcudnn8-dev \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Python packages for benchmarking, evaluation, and data download
# Pin numpy<2 because ros-humble-cv-bridge ships a cv2 compiled against NumPy 1.x
RUN pip3 install --no-cache-dir \
    evo \
    matplotlib \
    "numpy<2" \
    scipy \
    gdown

# Build Ceres Solver 2.2.0 from source (camera_models needs Manifold API, not in apt's 2.0)
RUN cd /tmp \
    && wget -q https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.2.0.tar.gz \
    && tar xzf 2.2.0.tar.gz \
    && cd ceres-solver-2.2.0 \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
    && make -j$(nproc) && make install \
    && rm -rf /tmp/ceres-solver-2.2.0 /tmp/2.2.0.tar.gz

# Download DBoW2 vocabulary + BRIEF pattern (static assets, ~58MB)
RUN mkdir -p /ros2_ws/support_files \
    && wget -q -O /ros2_ws/support_files/brief_k10L6.bin \
       https://github.com/zinuok/VINS-Fusion-ROS2/raw/main/support_files/brief_k10L6.bin \
    && wget -q -O /ros2_ws/support_files/brief_pattern.yml \
       https://github.com/zinuok/VINS-Fusion-ROS2/raw/main/support_files/brief_pattern.yml

# ONNX Runtime 1.17.1 GPU (C++ library for SuperPoint inference, supports CUDA 11.8)
RUN cd /ros2_ws \
    && wget -q https://github.com/microsoft/onnxruntime/releases/download/v1.17.1/onnxruntime-linux-x64-gpu-1.17.1.tgz \
    && tar xzf onnxruntime-linux-x64-gpu-1.17.1.tgz \
    && rm onnxruntime-linux-x64-gpu-1.17.1.tgz

# Python packages for SuperPoint ONNX model export (only needed at build time)
RUN pip3 install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cu118 \
    && pip3 install --no-cache-dir onnx lightglue

# Fix: create cv_bridge.hpp shim if not present (older Humble releases only have .h)
RUN if [ ! -f /opt/ros/humble/include/cv_bridge/cv_bridge/cv_bridge.hpp ]; then \
      echo '#include "cv_bridge/cv_bridge.h"' > /opt/ros/humble/include/cv_bridge/cv_bridge/cv_bridge.hpp; \
    fi

# Create workspace and clone SLAM packages
RUN mkdir -p /ros2_ws/src

# OpenVINS fork with challenge fixes + loop_fusion + camera_models
RUN cd /ros2_ws/src \
    && git clone --depth 1 https://github.com/eborghi10/open_vins.git

# Copy challenge tools into workspace
COPY . /ros2_ws/src/hilti-trimble-slam-challenge-2026

# Export SuperPoint ONNX model (backbone only: dense scores + descriptors)
RUN python3 /ros2_ws/src/hilti-trimble-slam-challenge-2026/scripts/export_superpoint_onnx.py \
    --output /ros2_ws/support_files/superpoint.onnx

# Build full workspace
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# --- Host user passthrough (preserves file permissions on mounted volumes) ---
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=user

RUN apt-get update && apt-get install -y --no-install-recommends sudo \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd --gid ${USER_GID} ${USERNAME} 2>/dev/null || true \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} 2>/dev/null || true \
    && mkdir -p /home/${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && chown -R ${USER_UID}:${USER_GID} /ros2_ws /home/${USERNAME}

# Source workspace on shell entry (for both root and the created user)
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=/ros2_ws/onnxruntime-linux-x64-gpu-1.17.1/lib:\$LD_LIBRARY_PATH" >> /root/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "export LD_LIBRARY_PATH=/ros2_ws/onnxruntime-linux-x64-gpu-1.17.1/lib:\$LD_LIBRARY_PATH" >> /home/${USERNAME}/.bashrc

ENV LD_LIBRARY_PATH="/ros2_ws/onnxruntime-linux-x64-gpu-1.17.1/lib:${LD_LIBRARY_PATH}"

# Data mount point
RUN mkdir -p /data && chown ${USER_UID}:${USER_GID} /data

USER ${USERNAME}
WORKDIR /ros2_ws
CMD ["/bin/bash"]
