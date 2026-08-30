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
    && apt-get clean

    
CMD ["/bin/bash"]

# Simulation layer
FROM dev-base as dev-sim
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
    > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y \
    libgz-transport13-dev \
    libgz-msgs10-dev \
    && apt-get clean

