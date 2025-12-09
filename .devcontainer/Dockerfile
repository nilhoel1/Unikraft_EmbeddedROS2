# Use ROS 2 Jazzy base image
FROM osrf/ros:jazzy-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install essential build tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    qemu-system-x86 \
    qemu-system-arm \
    qemu-utils \
    socat \
    gdb \
    apt-transport-https \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# Install Bazel
RUN curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor >bazel-archive-keyring.gpg && \
    mv bazel-archive-keyring.gpg /usr/share/keyrings/bazel-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list && \
    apt-get update && apt-get install -y bazel && \
    rm -rf /var/lib/apt/lists/*

# Install Buildifier (Bazel formatter, optional but helpful)
RUN BUILDIFIER_VERSION=6.4.0 && \
    wget https://github.com/bazelbuild/buildtools/releases/download/v${BUILDIFIER_VERSION}/buildifier-linux-amd64 -O /usr/bin/buildifier && \
    chmod +x /usr/bin/buildifier

# Install KraftKit
RUN wget https://get.kraftkit.sh -O /tmp/install-kraftkit.sh && \
    sh /tmp/install-kraftkit.sh -y && \
    rm /tmp/install-kraftkit.sh

# Add kraft to PATH
ENV PATH="/root/.local/bin:${PATH}"

# Install Rust (required for building Zenoh)
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Initialize rosdep
RUN rosdep update

# Set up ROS 2 workspace
RUN mkdir -p /workspace/src

# Set working directory
WORKDIR /workspace

# Source ROS 2 setup
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Default command
CMD ["/bin/bash"]
