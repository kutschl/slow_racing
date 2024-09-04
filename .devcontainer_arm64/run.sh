#!/bin/bash

# Get the directory of the script
SCRIPT_DIR=$(dirname "$0")
# Get the parent directory of the script directory
PROJECT_DIR=$(cd "$SCRIPT_DIR/.." && pwd)

# Parameters for the Docker build
USERNAME=${USER}
DOCKER_REPO="arm64v8/ros"
ROS_DISTRO="humble"
IMAGE_SUFFIX="-ros-base"
REPO_NAME="ss24_racing1"

# Define the Docker image name
IMAGE_NAME="${DOCKER_REPO}:${ROS_DISTRO}${IMAGE_SUFFIX}-${REPO_NAME}"

# Run the Docker container
docker run -it --rm --name ss24_racing1_container \
  --privileged \
  -v $PROJECT_DIR:/home/ss24_racing1 \
  -v /dev/sensors:/dev/sensors \
  -v /dev/input:/dev/input \
  -v /dev:/dev \
  -w /home/ss24_racing1 \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/dri:/dev/dri \
  -v $HOME/.ssh:/home/$USERNAME/.ssh \
  $IMAGE_NAME
