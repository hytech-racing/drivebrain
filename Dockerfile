# Builds for the host's arch (amd64 on CI, arm64 on Apple Silicon). Every Jetson/Phoenix
# dependency below is an arm64 .deb unpacked into a sysroot, so nothing needs an amd64 host
FROM ubuntu:22.04 as dev-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    make \
    cmake \
    build-essential \
    git \
    wget \
    curl \
    vim \
    pkg-config \
    python3 \
    python3-pip \
    python3-setuptools \
    python3-venv \
    binutils-aarch64-linux-gnu \
    patchelf \
    && apt-get clean

# Cross-compilation CUDA setup
ARG NV_REPO=https://repo.download.nvidia.com/jetson/common/pool/main
ARG CUDA=13-2_13.2.86-1
ARG TRT=10.16.2.10-1+cuda13.2
ARG PHOENIX6=26.3.0 

ENV JETSON_SYSROOT=/opt/jetson-sysroot

WORKDIR /tmp/jetson-debs

RUN wget -q ${NV_REPO}/c/cuda-cudart/cuda-cudart-${CUDA}_arm64.deb
RUN wget -q ${NV_REPO}/c/cuda-cudart/cuda-cudart-dev-${CUDA}_arm64.deb
RUN wget -q ${NV_REPO}/c/cuda-cudart/cuda-driver-dev-${CUDA}_arm64.deb
RUN wget -q ${NV_REPO}/c/cuda-crt/cuda-crt-${CUDA}_arm64.deb
RUN wget -q ${NV_REPO}/c/cuda-cccl/cuda-cccl-${CUDA}_arm64.deb

RUN wget -q ${NV_REPO}/t/tensorrt/libnvinfer10_${TRT}_arm64.deb
RUN wget -q ${NV_REPO}/t/tensorrt/libnvinfer-dev_${TRT}_arm64.deb
RUN wget -q ${NV_REPO}/t/tensorrt/libnvinfer-headers-dev_${TRT}_arm64.deb
RUN wget -q ${NV_REPO}/t/tensorrt/libnvinfer-plugin10_${TRT}_arm64.deb
RUN wget -q ${NV_REPO}/t/tensorrt/libnvinfer-plugin-dev_${TRT}_arm64.deb
RUN wget -q ${NV_REPO}/t/tensorrt/libnvinfer-headers-plugin-dev_${TRT}_arm64.deb

RUN wget -q -U "Debian APT-HTTP/1.3" \
    https://deb.ctr-electronics.com/libs/2026/packages/phoenix6/${PHOENIX6}/phoenix6_${PHOENIX6}_arm64.deb

RUN mkdir -p ${JETSON_SYSROOT} \
    && for deb in *.deb; do dpkg-deb -x "$deb" ${JETSON_SYSROOT}; done \
    && rm -rf /tmp/jetson-debs \
        ${JETSON_SYSROOT}/usr/share/doc \
        ${JETSON_SYSROOT}/usr/lib/aarch64-linux-gnu/*_static.a \
        ${JETSON_SYSROOT}/usr/lib/phoenix6/cmake \
        ${JETSON_SYSROOT}/usr/lib/phoenix6/*_Sim.so \
        ${JETSON_SYSROOT}/usr/lib/phoenix6/*_Replay.so

WORKDIR /
    
CMD ["/bin/bash"]

# Simulation layer
FROM dev-base as dev-sim
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
    > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y \
    libgz-transport13-dev \
    libgz-msgs10-dev \
    && apt-get clean

