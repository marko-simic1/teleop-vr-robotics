# ROS 2 & Docker Setup Guide

This repository contains the ROS 2 backend for the Teleoperation VR Robot project. It uses Docker to ensure a consistent development environment.

## Prerequisites
Before starting, ensure you have the following installed on your host machine (Ubuntu):
- **Git**
- **Docker**

To verify installation:
git --version
docker --version

1. Setup & Installation
Clone the repository

git clone https://github.com/marko-simic1/teleop-vr-robotics.git
cd teleop-vr-robotics

Build the Docker Image
You only need to do this once (or if you modify the Dockerfile).
docker build -t dipl-proj .

2. Running the Environment
We use a helper script run_docker.sh to start the container. This script automatically mounts your local code (ubuntu/src) into the Docker container. This means any change you make to the files in ubuntu/src will be instantly visible inside Docker.

Start the Container

# Make the script executable (only needed once)
chmod +x run_docker.sh

# Run the container
./run_docker.sh

Note: This script removes any old container named dipl-proj-container and starts a fresh one.

3. Building the ROS 2 Workspace
Once you are inside the Docker container shell (root@hostname:...), you must build the workspace to recognize your packages.

# 1. Go to the workspace root
cd /root/ros2_ws

# 2. Build the packages (ROS-TCP-Endpoint and diplomski_robot)
colcon build

# 3. Source the environment
source install/setup.bash


4. Running the Nodes
To run the full system, you will likely need two separate terminals inside the container.

Terminal 1: Unity Connection (TCP Endpoint)
Inside the container you just started:

ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
Terminal 2: Robot Logic (Navigation & Safety)

Open a new terminal window on your Ubuntu machine and connect to the already running container:
docker exec -it dipl-proj-container bash

Then, inside this new terminal:

cd /root/ros2_ws
source install/setup.bash

# Run your main navigation node
ros2 run diplomski_robot point_and_go_node




