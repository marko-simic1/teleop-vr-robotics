#!/bin/bash

# Remove old container if it exists
docker rm -f dipl-proj-container 2>/dev/null

# Allow Docker GUI access
xhost +local:docker

docker run -it \
  --name dipl-proj-container \
  --net=host \
  --privileged \
  --env DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/ros2_ws:/root/ros2_ws \
  dipl-proj
