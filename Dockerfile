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
    vim \
    pkg-config \
    python3 \
    python3-pip \
    python3-setuptools \
    python3-venv \
    ca-certificates \
    && apt-get clean

    
CMD ["/bin/bash"]

# Jetson Orin AGX deps

FROM dev-base AS dev-inference
ARG CUDA_TOOLKIT_VERSION=12-5
ARG CUDA_TOOLKIT_PACKAGE_VERSION=12.5.1-1
ARG TENSORRT_VERSION=10.3.0.26-1+cuda12.5
RUN test "$(dpkg --print-architecture)" = amd64 \
    && wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb -O /tmp/cuda-keyring.deb \
    && echo 'd93190d50b98ad4699ff40f4f7af50f16a76dac3bb8da1eaaf366d47898ff8df  /tmp/cuda-keyring.deb' | sha256sum -c - \
    && dpkg -i /tmp/cuda-keyring.deb \
    && rm /tmp/cuda-keyring.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       "cuda-toolkit-${CUDA_TOOLKIT_VERSION}=${CUDA_TOOLKIT_PACKAGE_VERSION}" \
       "libnvinfer10=${TENSORRT_VERSION}" \
       "libnvinfer-headers-dev=${TENSORRT_VERSION}" \
       "libnvinfer-dev=${TENSORRT_VERSION}" \
       "libnvonnxparsers10=${TENSORRT_VERSION}" \
       "libnvonnxparsers-dev=${TENSORRT_VERSION}" \
    && apt-mark hold libnvinfer10 libnvinfer-headers-dev libnvinfer-dev libnvonnxparsers10 libnvonnxparsers-dev \
    && rm -rf /var/lib/apt/lists/*
ENV PATH="/usr/local/cuda/bin:${PATH}"

