FROM ros:humble-desktop

# Prevent interactive tzdata
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    curl gnupg2 ca-certificates lsb-release \
    build-essential cmake git \
    python3-pip python3-colcon-common-extensions python3-colcon-ros \
    clang-format clang-tidy cppcheck ccache \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Node.js 18
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - \
    && apt-get update && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directories
WORKDIR /workspaces/CARE

# Default shell setup for ROS 2
SHELL ["/bin/bash", "-lc"]
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
RUN echo "export CCACHE_DIR=/root/.ccache" >> /etc/bash.bashrc \
    && echo "export CC='ccache gcc' CXX='ccache g++'" >> /etc/bash.bashrc \
    && echo "export PATH=\"/usr/lib/ccache:$PATH\"" >> /etc/bash.bashrc


