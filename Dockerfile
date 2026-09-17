FROM ubuntu:22.04

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

RUN dpkg --add-architecture arm64 \
    && curl -fsSL --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg" \
    && echo "deb [signed-by=/usr/share/keyrings/ctr-pubkey.gpg] https://deb.ctr-electronics.com/libs/2026 stable main" > /etc/apt/sources.list.d/ctr2026.list \
    && apt-get -o Dir::Etc::sourcelist="/etc/apt/sources.list.d/ctr2026.list" -o Dir::Etc::sourceparts="/dev/null" -o APT::Get::List-Cleanup="0" update \
    && apt-get install -y phoenix6:arm64 \
    && apt-get clean \
    && sed -i \
        -e 's#set(phoenix6_LIBRARIES "-lCTRE_Phoenix6 -lCTRE_PhoenixTools")#set(phoenix6_LIBRARIES "${libdir}/libCTRE_Phoenix6.so;${libdir}/libCTRE_PhoenixTools.so")#' \
        -e 's#INTERFACE_LINK_LIBRARIES \${phoenix6_LIBRARIES})#INTERFACE_LINK_LIBRARIES "${phoenix6_LIBRARIES}")#' \
        /usr/lib/phoenix6/cmake/phoenix6-config.cmake

CMD ["/bin/bash"]