#!/bin/bash
#
# Copyright (c) 2023, Mohamed Abdelkader.  All rights reserved.


ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/../utils/print_color.sh
source $ROOT/../utils/l4t_version.sh

SETUP_ISAAC_ROS="True"

# Prevent running as root.
if [[ $(id -u) -eq 0 ]]; then
    print_error "This script cannot be executed with root privileges."
    print_error "Please re-run without sudo and follow instructions to configure docker for non-root user if needed."
    exit 1
fi

# Check if user can run docker without root.
RE="\<docker\>"
if [[ ! $(groups $USER) =~ $RE ]]; then
    print_error "User |$USER| is not a member of the 'docker' group and cannot run docker commands without sudo."
    print_error "Run 'sudo usermod -aG docker \$USER && newgrp docker' to add user to 'docker' group, then re-run this script."
    print_error "See: https://docs.docker.com/engine/install/linux-postinstall/"
    exit 1
fi

# Check if able to run docker commands.
if [[ -z "$(docker ps)" ]] ;  then
    print_error "Unable to run docker commands. If you have recently added |$USER| to 'docker' group, you may need to log out and log back in for it to take effect."
    print_error "Otherwise, please check your Docker installation."
    exit 1
fi

PLATFORM="$(uname -m)"

BASE_NAME="mzahana/d2dtracker-jetson:r${L4T_VERSION}"
CONTAINER_NAME="d2dtracker-container"


CMD="export DEV_DIR=\$HOME/shared_volume && \
        if [[ -f "\$HOME/shared_volume/ros2_ws/install/setup.bash" ]]; then
            source \$HOME/shared_volume/ros2_ws/install/setup.bash
        fi && \
         /bin/bash"
if [[ -n "$GIT_TOKEN" ]] && [[ -n "$GIT_USER" ]]; then
    CMD="export GIT_USER=$GIT_USER && export GIT_TOKEN=$GIT_TOKEN && $CMD"
fi

if [[ -n "$SUDO_PASSWORD" ]]; then
    CMD="export SUDO_PASSWORD=$SUDO_PASSWORD && $CMD"
fi

if [[ -n "$SETUP_ISAAC_ROS" ]]; then
    CMD="export SETUP_ISAAC_ROS=$SETUP_ISAAC_ROS && $CMD"
fi


if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    if [ "$(docker ps -aq -f status=exited -f name=${CONTAINER_NAME})" ]; then
        # cleanup
        echo "Restarting the container..."
        docker start ${CONTAINER_NAME}
    fi
    docker exec -it --workdir /root/shared_volume ${CONTAINER_NAME} env TERM=xterm-256color bash -c "${CMD}"
    exit 0
fi

DISPLAY_DEVICE=" "
DOCKER_ARGS=" "

if [ -n "$DISPLAY" ]; then
	# give docker root user X11 permissions
	sudo xhost +si:localuser:root
	
	# enable SSH X11 forwarding inside container (https://stackoverflow.com/q/48235040)
	XAUTH=/tmp/.docker.xauth
	sudo xauth nlist $DISPLAY | sudo sed -e 's/^..../ffff/' | sudo xauth -f $XAUTH nmerge -
	sudo chmod 777 $XAUTH

	DISPLAY_DEVICE="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH"
	DOCKER_ARGS+=("-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH")
fi


# Map host's display socket to docker
#DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
#DOCKER_ARGS+=("-v $HOME/.Xauthority:/root/.Xauthority:rw")
#DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml")

if [[ $PLATFORM == "aarch64" ]]; then
    DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats")
    DOCKER_ARGS+=("-v /tmp/argus_socket:/tmp/argus_socket")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h:/usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h")
    DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
    DOCKER_ARGS+=("-v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
    DOCKER_ARGS+=("-v /opt/nvidia/nsight-systems-cli:/opt/nvidia/nsight-systems-cli")
    DOCKER_ARGS+=("--pid=host")
    DOCKER_ARGS+=("-v /opt/nvidia/vpi2:/opt/nvidia/vpi2")
    DOCKER_ARGS+=("-v /usr/share/vpi2:/usr/share/vpi2")

    # If jtop present, give the container access
    if [[ $(getent group jtop) ]]; then
        DOCKER_ARGS+=("-v /run/jtop.sock:/run/jtop.sock:ro")
        JETSON_STATS_GID="$(getent group jtop | cut -d: -f3)"
        DOCKER_ARGS+=("--group-add $JETSON_STATS_GID")
    fi
fi


# Custom command to run after logging into the container
CMD="export DEV_DIR=\$HOME/shared_volume && \
        if [ ! -d "\$HOME/shared_volume/ros2_ws" ]; then
            mkdir -p \$HOME/shared_volume/ros2_ws/src
        fi && \
        if [ -d "\$HOME/shared_volume/ros2_ws/install" ]; then
            source \$HOME/shared_volume/ros2_ws/install/setup.bash
        fi && \
        /bin/bash"

if [[ -n "$GIT_TOKEN" ]] && [[ -n "$GIT_USER" ]]; then
    CMD="export GIT_USER=$GIT_USER && export GIT_TOKEN=$GIT_TOKEN && $CMD"
fi

if [[ -n "$SUDO_PASSWORD" ]]; then
    CMD="export SUDO_PASSWORD=$SUDO_PASSWORD && $CMD"
fi

if [[ -n "$SETUP_ISAAC_ROS" ]]; then
    CMD="export SETUP_ISAAC_ROS=$SETUP_ISAAC_ROS && $CMD"
fi

HOST_DEV_DIR=$HOME/${CONTAINER_NAME}_shared_volume
if [ ! -d "$HOST_DEV_DIR" ]; then
    mkdir -p $HOST_DEV_DIR
fi

# Run container from image
print_info "Running $CONTAINER_NAME"
docker run -it \
    --privileged \
    --network host \
    ${DOCKER_ARGS[@]} \
    -v $HOST_DEV_DIR:/root/shared_volume \
    -v /dev/*:/dev/* \
    -v /etc/localtime:/etc/localtime:ro \
    --name "$CONTAINER_NAME" \
    --runtime nvidia \
    --entrypoint /ros_entrypoint.sh \
    --workdir /root/shared_volume \
    $@ \
    $BASE_NAME \
    bash -c "${CMD}"