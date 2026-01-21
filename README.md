# VR Teleoperation & Semi-Autonomous Robot Navigation

This repository contains the complete system for controlling a mobile robot (AgileX Ranger Mini) using Virtual Reality (Meta Quest 3). It implements a "Digital Twin" concept where the ROS 2 backend handles navigation/safety, and the Unity frontend handles the VR interface.

## Repository Structure

* **`ubuntu/` (Backend):** Contains the ROS 2 Humble source code.
    * Designed to run inside a Docker container.
    * Handles navigation (`point_and_go`), safety filters, and hardware communication.
* **`unity/` (Frontend):** Contains the Unity VR project.
    * Visualizes the robot, processes VR controller inputs, and communicates with ROS via TCP.
* **`docker/` or root:** Contains setup scripts for the development environment.

---

## Part 1: ROS 2 Backend (Ubuntu)

### Prerequisites
* Docker installed on Linux.
* Git.

### 1. Setup
Clone the repo and build the Docker image (only needed once):
```
git clone [https://github.com/marko-simic1/teleop-vr-robotics.git](https://github.com/marko-simic1/teleop-vr-robotics.git)
cd teleop-vr-robotics
docker build -t dipl-proj .
```

# 2. Running the Container
We use a helper script to mount the local code (ubuntu/src) into Docker so changes are applied instantly.

```
chmod +x run_docker.sh
./run_docker.sh
```

# 3. Building the Workspace
Inside the container terminal:
```
cd /root/ros2_ws
colcon build
source install/setup.bash
```

## Part 2: Unity VR Frontend (Windows)
Prerequisites
- Unity Hub & Unity Editor (2022.3 LTS recommended).
- Meta Quest Link App (for Quest 3).

# 1. Open the Project
Open Unity Hub.
Click Add -> Add project from disk.
Select the unity folder inside this repository.

# 2. Configure Connection
The Unity project needs to know the IP address of your Ubuntu machine (where Docker is running).

On Ubuntu, find your IP:
```
hostname -I
```
(Look for the local IP, usually something like 192.168.1.x)

In Unity Editor:
  Go to menu: Robotics -> ROS Settings.
  ROS IP Address: Enter your Ubuntu IP.
  ROS Port: 10000.
  Protocol: ROS2.

## Part 3: Running the Full System
To operate the robot, follow this sequence:

Step 1: Start Communication (Ubuntu/Docker)
Inside the Docker container:
```
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```
You should see: "Starting server on port 10000..."

Step 2: Start Robot Logic (Ubuntu/Docker)
Open a new terminal, connect to the container, and run the navigation logic:
```
docker exec -it dipl-proj-container bash
source /root/ros2_ws/install/setup.bash
ros2 run diplomski_robot point_and_go_node
```

Step 3: Start VR (Windows)
  Connect your Meta Quest 3 via Link (Cable or AirLink).
  Press Play in Unity.
  If the connection is successful, the HUD in the top-left corner will turn blue/green.



