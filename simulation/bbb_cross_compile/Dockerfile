# Dockerfile for BeagleBone Black Cross-Compilation
# This provides a complete Linux environment with ARM cross-compiler
# Using Ubuntu 20.04 for better compatibility with older glibc on BBB

FROM ubuntu:20.04

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install basic tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    python3 \
    python3-pip \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Install ARM cross-compiler for Ubuntu 20.04
RUN apt-get update && apt-get install -y \
    gcc-arm-linux-gnueabihf \
    g++-arm-linux-gnueabihf \
    libc6-dev-armhf-cross \
    && rm -rf /var/lib/apt/lists/*

# Enable multiarch for ARM packages
RUN dpkg --add-architecture armhf

# Install ZeroMQ development files for ARM cross-compilation
RUN apt-get update && apt-get install -y \
    libzmq3-dev:armhf \
    && rm -rf /var/lib/apt/lists/*

# Install Eigen3 (header-only library)
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Verify installation
RUN arm-linux-gnueabihf-gcc --version
RUN arm-linux-gnueabihf-g++ --version
RUN cmake --version

# Create working directory
WORKDIR /workspace

# Default command
CMD ["/bin/bash"]