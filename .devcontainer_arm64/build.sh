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

echo "DOCKER_REPO=$DOCKER_REPO"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "IMAGE_SUFFIX=$IMAGE_SUFFIX"

# Define the Docker image name
IMAGE_NAME="${DOCKER_REPO}:${ROS_DISTRO}${IMAGE_SUFFIX}-${REPO_NAME}"

# Execute the Docker build
docker build -t $IMAGE_NAME \
    --build-arg DOCKER_REPO=$DOCKER_REPO \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    --build-arg IMAGE_SUFFIX=$IMAGE_SUFFIX \
    --build-arg USERNAME=$USERNAME \
    --build-arg USER_UID=$(id -u) \
    --build-arg USER_GID=$(id -g) \
    -f $PROJECT_DIR/.devcontainer_arm64/Dockerfile_arm64 $PROJECT_DIR

echo "Docker image $IMAGE_NAME built successfully."