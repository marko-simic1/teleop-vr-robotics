# -------------------------------------------------------
# Base image: ROS2 Humble on Ubuntu 22.04
# -------------------------------------------------------
FROM osrf/ros:humble-desktop-full

# Avoid prompts during install
ENV DEBIAN_FRONTEND=noninteractive

# -------------------------------------------------------
# Install essential tools + Git + Git LFS + SSH
# -------------------------------------------------------
RUN apt-get update && apt-get install -y \
    git \
    git-lfs \
    openssh-client \
    curl \
    wget \
    nano \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Enable Git LFS
RUN git lfs install

# -------------------------------------------------------
# Set up ROS environment (auto-source)
# -------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# -------------------------------------------------------
# Optional: Create workspace
# -------------------------------------------------------
RUN mkdir -p /root/ros2_ws/src

WORKDIR /root/ros2_ws

# -------------------------------------------------------
# (Optional) Pre-install some Python libs
# -------------------------------------------------------
RUN pip3 install --no-cache-dir \
    numpy \
    open3d \
    rospkg

# -------------------------------------------------------
# Default command
# -------------------------------------------------------
CMD ["bash"]
