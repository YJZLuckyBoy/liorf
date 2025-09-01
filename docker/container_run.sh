#!/bin/bash
# Author : Taeyoung Kim (https://github.com/Taeyoung96)

# Set the project directory (PROJECT_DIR) as the parent directory of the current working directory
PROJECT_DIR=$(dirname "$PWD")

# Move to the parent folder of the project directory
cd "$PROJECT_DIR"

# Print the current working directory to verify the change
echo "Current working directory: $PROJECT_DIR"

# Check if arguments are provided for the image name and tag
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <container_name> <image_name:tag>"
  exit 1
fi

# Assign the arguments to variables for clarity
CONTAINER_NAME="$1"
IMAGE_NAME="$2"

# Launch the nvidia-docker container with the provided image name and tag
docker run --privileged -it \
            --gpus all \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e NVIDIA_VISIBLE_DEVICES=all \
           --volume="$PROJECT_DIR:/root/ros2_ws/src" \
           --volume=/data/LIDAR_dataset:/root/data \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --net=host \
           --ipc=host \
           --shm-size=4gb \
           --name="$CONTAINER_NAME" \
	         -v /home/iit.local/ynistico/dls_ws_home/rosbags/outdoor_exp_3_0927_with_revised_all:/rosbags \
           --env="DISPLAY=$DISPLAY" \
           "$IMAGE_NAME" /bin/bash
