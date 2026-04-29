# syntax=docker/dockerfile:1.4
FROM debian:bookworm-slim
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config python3 python3-pip \
    gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
    libcamera-dev libcamera-apps \
    python3-picamera2 python3-opencv python3-numpy \
    ffmpeg gstreamer1.0-tools v4l-utils python3-serial sudo \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -m -u 1000 -s /bin/bash dev && \
    echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    usermod -aG video,dialout dev

USER dev
WORKDIR /workspace
