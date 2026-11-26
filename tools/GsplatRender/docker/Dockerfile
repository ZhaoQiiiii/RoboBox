# GsplatRender 独立镜像：同一容器内运行 TCP 3DGS renderer + ROS 1 bridge。
#
# 构建（在 tools/GsplatRender 目录执行，context 必须是本目录）：
#   docker build -f docker/Dockerfile -t gsplat-render:noetic .
#
# 运行（需要宿主 NVIDIA 驱动 + nvidia-container-toolkit，以及可达的 ROS master）：
#   docker run --gpus all --net=host \
#     -v /path/to/scene.ply:/data/scene.ply:ro \
#     gsplat-render:noetic /data/scene.ply
#
# 第一个参数是 PLY 路径，其余参数原样传给 renderer。云台模式默认开启：
#   gsplat-render:noetic /data/scene.ply
#
# gsplat 从源码编译，构建时间主要在这里；只要目标机器架构时可加速：
#   --build-arg TORCH_CUDA_ARCH_LIST=8.6
#
# 依赖版本与 docker/Dockerfile.base 保持一致：torch 2.7.0+cu128、gsplat 1.5.3
# 源码编译、numpy 1.26.4、ROS Noetic + cv_bridge。

# =====================================================================
# Stage 1: builder —— 编译 gsplat，产出一个可整体复制的 Python 3.10 venv
# =====================================================================
FROM nvidia/cuda:12.8.1-devel-ubuntu20.04 AS builder

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
# Stage 2: runtime —— ROS Noetic + Python 3.8 bridge + 复制 venv
# =====================================================================
FROM nvidia/cuda:12.8.1-runtime-ubuntu20.04 AS runtime

ARG HTTP_PROXY HTTPS_PROXY http_proxy https_proxy NO_PROXY no_proxy
ARG UBUNTU_MIRROR=mirrors.tuna.tsinghua.edu.cn
ARG ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    DISABLE_ROS1_EOL_WARNINGS=1 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility

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

# bridge 只需要 rospy / cv_bridge / sensor_msgs / nav_msgs，不需要 desktop/rviz。
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

# 构建期确认 bridge 的 ROS 依赖在系统 Python 3.8 下可导入。
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && \
    /usr/bin/python3 -c "import rospy, cv_bridge; from sensor_msgs.msg import Image; from nav_msgs.msg import Odometry; print(\"bridge deps ok\")"'

# Python 3.10 + 已编译的 torch/gsplat，不含 nvcc/编译器。
COPY --from=builder /opt/uv-python /opt/uv-python
COPY --from=builder /opt/gsplat-venv /opt/gsplat-venv

WORKDIR /opt/gsplat-render
COPY runtime/ ./runtime/
COPY config/ ./config/
COPY scripts/ ./scripts/
COPY docker/entrypoint.sh ./docker/entrypoint.sh
COPY README.md LICENSE ./
RUN chmod 0755 ./docker/entrypoint.sh

EXPOSE 19080 19081

ENTRYPOINT ["/opt/gsplat-render/docker/entrypoint.sh"]
CMD ["/data/scene.ply"]
