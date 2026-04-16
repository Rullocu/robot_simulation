#!/bin/bash
set -e

# Configuration
IMAGE_NAME="ros_humble"
CONTAINER_NAME="ros_humble"

# Function to print colored output
print_colored() {
    echo -e "\e[1;34m$1\e[0m"
}

# Check if image exists, build if it doesn't
if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
    print_colored "Building Docker image: $IMAGE_NAME"

    TMP_DIR=$(mktemp -d)
    cp Dockerfile $TMP_DIR/
    cp entrypoint.sh $TMP_DIR/

    docker build -f Dockerfile -t $IMAGE_NAME $TMP_DIR

    rm -rf $TMP_DIR
else
    print_colored "Docker image $IMAGE_NAME already exists"
fi

# Stop and remove any existing container with the same name
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    print_colored "Stopping existing container: $CONTAINER_NAME"
    docker stop $CONTAINER_NAME
    docker rm $CONTAINER_NAME
fi

if [ "$(docker ps -aq -f status=exited -f name=$CONTAINER_NAME)" ]; then
    print_colored "Removing stopped container: $CONTAINER_NAME"
    docker rm $CONTAINER_NAME
fi

# Create workspace directories if they don't exist
print_colored "Ensuring workspace directories exist"
mkdir -p "$(pwd)/src"
mkdir -p "$(pwd)/build"
mkdir -p "$(pwd)/install"
mkdir -p "$(pwd)/log"

# WSL2 uses WSLg for GUI — no xhost needed, just mount the WSLg sockets
print_colored "Using WSLg for GUI forwarding (software rendering, no GPU)"

# Run the container
docker run -it --rm \
    --name $CONTAINER_NAME \
    --network=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir \
    -e PULSE_SERVER=$PULSE_SERVER \
    -e ROS_DOMAIN_ID=42 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e MESA_GL_VERSION_OVERRIDE=3.3 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /mnt/wslg:/mnt/wslg \
    -v "$(pwd)/src:/ros2_ws/src" \
    -v "$(pwd)/build:/ros2_ws/build" \
    -v "$(pwd)/install:/ros2_ws/install" \
    -v "$(pwd)/log:/ros2_ws/log" \
    -v /dev:/dev \
    $IMAGE_NAME "$@"

print_colored "Container execution completed"
