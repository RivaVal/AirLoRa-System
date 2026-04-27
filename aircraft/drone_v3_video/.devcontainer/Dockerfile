FROM debian:bookworm-slim
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8FROM debian:bookworm-slim
LABEL maintainer="your@email.com"
LABEL description="RPi Zero 2W + ESP32 Drone Dev Env"

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# === Установка ВСЕХ зависимостей ===
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl python3 python3-pip python3-serial \
    gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
    pkg-config \
    libcamera-dev libcamera-apps \
    libopencv-dev python3-opencv python3-numpy \
    ffmpeg gstreamer1.0-tools v4l-utils \
    sudo locales \
    && rm -rf /var/lib/apt/lists/*

# === Локаль ===
RUN sed -i 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen en_US.UTF-8

# === Пользователь 'dev' ===
RUN useradd -m -u 1000 -s /bin/bash dev && \
    echo "dev:dev" | chpasswd && \
    adduser dev sudo && \
    echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    usermod -aG video,dialout dev && \
    mkdir -p /workspace/src /workspace/build /workspace/archive && \
    chown -R dev:dev /workspace

USER dev
WORKDIR /workspace

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config python3 python3-pip \
    pkg-config \  # ← ДОБАВЛЕНО: обязательно для CMake + pkg_check_modules
    gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
    libcamera-dev libcamera-apps \
    python3-picamera2 python3-opencv python3-numpy \
    libopencv-dev \  # ← ДОБАВЛЕНО: C++ заголовки и .cmake файлы для OpenCV
    ffmpeg gstreamer1.0-tools v4l-utils python3-serial sudo \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -m -u 1000 -s /bin/bash dev && \
    echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    usermod -aG video,dialout dev

USER dev
WORKDIR /workspace

