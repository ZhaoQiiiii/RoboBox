# =====================================================================
# RoboKit tools image: robokit:latest
#
# 一个镜像同时包含 GsplatRender 与 LidarRender：
#   docker run robokit:latest gsplat /data/scene.ply
#   docker run robokit:latest lidar  /data/scene.pcd
#
# Build context: runtime/RoboKit
# =====================================================================

# =====================================================================
# Stage 1: gsplat builder —— 编译 gsplat，产出一个可整体复制的 Python 3.10 venv
# =====================================================================
FROM nvidia/cuda:12.8.1-devel-ubuntu20.04 AS gsplat-builder

LABEL maintainer="iinnoovo123@gmail.com"

ARG HTTP_PROXY HTTPS_PROXY http_proxy https_proxy NO_PROXY no_proxy
ARG UBUNTU_MIRROR=mirrors.tuna.tsinghua.edu.cn
ARG TORCH_CUDA_ARCH_LIST="7.0;7.5;8.0;8.6;8.9;9.0;10.0;12.0+PTX"
ARG MAX_JOBS=8

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1

RUN sed -i "s|archive.ubuntu.com|${UBUNTU_MIRROR}|g; s|security.ubuntu.com|${UBUNTU_MIRROR}|g" /etc/apt/sources.list && \
    rm -f /etc/apt/sources.list.d/cuda.list /etc/apt/sources.list.d/nvidia-ml.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      git \
      build-essential \
      ninja-build \
      pkg-config && \
    rm -rf /var/lib/apt/lists/*

# Python 3.10 由 uv 安装到固定路径，运行时阶段原样复制，venv 的符号链接不会断。
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:${PATH}"

RUN uv python install 3.10 && \
    uv venv --python 3.10 /opt/gsplat-venv

ENV CUDA_HOME=/usr/local/cuda \
    FORCE_CUDA=1 \
    MAX_JOBS=${MAX_JOBS} \
    TORCH_CUDA_ARCH_LIST=${TORCH_CUDA_ARCH_LIST}

# renderer 运行时只需要 torch/gsplat + 数值/IO 依赖。
# 显式锁定 +cu128 本地版本，避免 uv 从 PyPI 解析到默认 CUDA 构建。
RUN uv pip install --python /opt/gsplat-venv/bin/python --no-cache \
      pip \
      setuptools \
      wheel \
      ninja \
      packaging && \
    uv pip install --python /opt/gsplat-venv/bin/python --no-cache \
      --extra-index-url https://download.pytorch.org/whl/cu128 \
      torch==2.7.0+cu128 && \
    uv pip install --python /opt/gsplat-venv/bin/python --no-cache \
      jaxtyping \
      rich \
      plyfile \
      scipy==1.15.3 \
      numpy==1.26.4 \
      PyYAML && \
    uv pip install --python /opt/gsplat-venv/bin/python --no-cache \
      --no-binary gsplat \
      --no-build-isolation \
      gsplat==1.5.3

# 构建期断言，防止基础镜像或索引把版本悄悄换掉。
RUN /opt/gsplat-venv/bin/python -c "\
from importlib.metadata import version; \
import torch; \
torch_version = version('torch'); \
numpy_version = version('numpy'); \
assert torch_version == '2.7.0+cu128', torch_version; \
assert torch.version.cuda == '12.8', torch.version.cuda; \
assert int(numpy_version.split('.')[0]) < 2, numpy_version; \
import gsplat, scipy, plyfile, yaml, numpy; \
print('renderer deps ok:', torch_version, 'numpy', numpy_version)"

# =====================================================================
# Stage 2: runtime base —— CUDA runtime + ROS Noetic bridge + EGL/GL
# =====================================================================
FROM nvidia/cuda:12.8.1-runtime-ubuntu20.04 AS robokit-runtime-base

LABEL maintainer="iinnoovo123@gmail.com"

ARG HTTP_PROXY HTTPS_PROXY http_proxy https_proxy NO_PROXY no_proxy
ARG UBUNTU_MIRROR=mirrors.tuna.tsinghua.edu.cn
ARG ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    DISABLE_ROS1_EOL_WARNINGS=1 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,display,compute,utility

RUN sed -i "s|archive.ubuntu.com|${UBUNTU_MIRROR}|g; s|security.ubuntu.com|${UBUNTU_MIRROR}|g" /etc/apt/sources.list && \
    rm -f /etc/apt/sources.list.d/cuda.list /etc/apt/sources.list.d/nvidia-ml.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      tzdata \
      ca-certificates \
      gnupg2 \
      lsb-release \
      curl \
      python3 \
      python3-numpy \
      libgomp1 && \
    rm -rf /var/lib/apt/lists/*

# bridge 需要 rospy / cv_bridge / sensor_msgs / nav_msgs，不需要 desktop/rviz。
# keyserver 不可达时回退到 rosdistro 的 ros.asc。
RUN (apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' \
       --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 || \
     curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -) && \
    echo "deb ${ROS_APT_MIRROR} $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-noetic-ros-base \
      ros-noetic-cv-bridge && \
    rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && \
    /usr/bin/python3 -c "import rospy, cv_bridge; from sensor_msgs.msg import Image; from nav_msgs.msg import Odometry; print(\"bridge deps ok\")"'

# LidarRender 的 C++ 渲染与全局点云 PCD 解析依赖。
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      libpcl-dev \
      libopencv-dev \
      libglfw3 \
      libyaml-cpp0.6 \
      libjsoncpp1 \
      libgl1 \
      libglx0 \
      libglvnd0 \
      libegl1 \
      libgl1-mesa-dri \
      libx11-6 \
      libxrandr2 \
      libxinerama1 \
      libxcursor1 \
      libxi6 \
      xvfb && \
    rm -rf /var/lib/apt/lists/*

# =====================================================================
# Stage 3: lidar builder —— C++ marsim_render + TCP renderer
# =====================================================================
FROM ubuntu:20.04 AS lidar-builder

ARG HTTP_PROXY HTTPS_PROXY http_proxy https_proxy NO_PROXY no_proxy
ARG UBUNTU_MIRROR=mirrors.tuna.tsinghua.edu.cn

ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i "s|archive.ubuntu.com|${UBUNTU_MIRROR}|g; s|security.ubuntu.com|${UBUNTU_MIRROR}|g" /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ca-certificates \
      build-essential \
      cmake \
      pkg-config \
      libeigen3-dev \
      libpcl-dev \
      libopencv-dev \
      libglfw3-dev \
      libyaml-cpp-dev \
      libjsoncpp-dev \
      libomp-dev \
      libglvnd-dev \
      libgl1-mesa-dev \
      libegl1-mesa-dev && \
    rm -rf /var/lib/apt/lists/*

COPY tools/LidarRender /src/LidarRender
RUN cmake -S /src/LidarRender -B /build \
      -DLIDAR_RENDER_ROOT_DIR=/opt/lidar-render/marsim_render/ \
      -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /build -j"$(nproc)"

# =====================================================================
# Stage 4: robokit:latest
# =====================================================================
FROM robokit-runtime-base AS robokit

COPY --from=gsplat-builder /opt/uv-python /opt/uv-python
COPY --from=gsplat-builder /opt/gsplat-venv /opt/gsplat-venv

WORKDIR /opt/gsplat-render
COPY tools/GsplatRender/runtime/ ./runtime/
COPY tools/GsplatRender/config/ ./config/
COPY tools/GsplatRender/scripts/ ./scripts/
COPY tools/GsplatRender/docker/entrypoint.sh ./docker/entrypoint.sh
COPY tools/GsplatRender/README.md tools/GsplatRender/LICENSE ./

COPY --from=lidar-builder /build/lidar_render_tcp /opt/lidar-render/bin/lidar_render_tcp
COPY tools/LidarRender/marsim_render/config/ /opt/lidar-render/marsim_render/config/
COPY tools/LidarRender/config/ /opt/lidar-render/config/
COPY tools/LidarRender/runtime/ /opt/lidar-render/runtime/
COPY tools/LidarRender/docker/entrypoint.sh /opt/lidar-render/docker/entrypoint.sh
COPY tools/LidarRender/README.md tools/LidarRender/README-SOURCE.md /opt/lidar-render/

COPY docker/entrypoint.sh /opt/robokit/docker/entrypoint.sh
RUN chmod 0755 /opt/robokit/docker/entrypoint.sh \
      /opt/gsplat-render/docker/entrypoint.sh \
      /opt/lidar-render/docker/entrypoint.sh

EXPOSE 19080 19081 19082 19083
ENTRYPOINT ["/opt/robokit/docker/entrypoint.sh"]
CMD ["gsplat", "/data/scene.ply"]
