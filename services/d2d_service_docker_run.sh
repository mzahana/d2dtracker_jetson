#!/usr/bin/env bash
# Non-interactive script to run Docker container with ROS2 by a systemd service
# This is run by the systemd service: docker-ros2-container.service
# This script assumes uses an entry point script named docker_entrypoint.sh that is located in the shared 
#   volume inside the container /root/shared_volume

# Log file location
LOG_FILE="/var/log/d2d_service_docker_run.log"

# Function to log messages
log_message() {
    echo "$(date): $1" >> $LOG_FILE
}

log_message "Starting Docker Run Script"

# Define full path for Docker
DOCKER_CMD="/usr/bin/docker"

# Root directory of the script
ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
log_message "Root directory: $ROOT"

# Check for V4L2 devices
V4L2_DEVICES=""
for i in {0..9}
do
    if [ -e "/dev/video$i" ]; then
        V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
    fi
done

# check for display
DISPLAY_DEVICE=""
# This is hardcoded as systemd services don't see the environment variables
# DISPLAY=:1
if [ -n "$DISPLAY" ]; then
	# give docker root user X11 permissions
	xhost +si:localuser:root
	
	# enable SSH X11 forwarding inside container (https://stackoverflow.com/q/48235040)
	XAUTH=/tmp/.docker.xauth
	xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
	chmod 777 $XAUTH

	DISPLAY_DEVICE="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH"
fi

log_message "V4L2 Devices: $V4L2_DEVICES"

# Define the container name
CONTAINER_NAME=d2dtracker-container

# Check the architecture
ARCH=$(uname -m)
log_message "Architecture: $ARCH"
IMAGE="mzahana/d2dtracker-jetson:r35.4.1"
DOCKER_USERNAME=d2d
HOST_USERNAME=hunter2

# Map host's display socket to docker
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/$DOCKER_USERNAME/.Xauthority:rw")
# DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")

if [[ "$ARCH" == "aarch64" ]]; then
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
    DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml")

    # If jtop present, give the container access
    if [[ $(getent group jtop) ]]; then
        DOCKER_ARGS+=("-v /run/jtop.sock:/run/jtop.sock:ro")
        JETSON_STATS_GID="$(getent group jtop | cut -d: -f3)"
        DOCKER_ARGS+=("--group-add $JETSON_STATS_GID")
    fi
fi


if [ "$ARCH" = "aarch64" ]; then
    log_message "Running on aarch64 architecture"

    $DOCKER_CMD run -p 12345:12345 -p 5000:5000 --runtime nvidia -d --rm --network host \
        --volume /etc/enctune.conf:/etc/enctune.conf \
        --volume /etc/nv_tegra_release:/etc/nv_tegra_release \
        --volume /home/$HOST_USERNAME/${CONTAINER_NAME}_shared_volume:/home/$DOCKER_USERNAME/shared_volume \
        --device /dev/snd \
        --device /dev/bus/usb \
        --entrypoint /home/$DOCKER_USERNAME/shared_volume/d2d_service_entrypoint.sh \
        -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        --name ${CONTAINER_NAME} \
        $DISPLAY_DEVICE \
        $DOCKER_ARGS \
        ${IMAGE} $V4L2_DEVICES >> $LOG_FILE 2>&1

elif [ "$ARCH" = "x86_64" ]; then
    log_message "Running on x86_64 architecture"

    $DOCKER_CMD run -p 12345:12345 -p 5000:5000 --gpus all -d --rm --network=host \
        --shm-size=8g \
        --ulimit memlock=-1 \
        --ulimit stack=67108864 \
        --env NVIDIA_DRIVER_CAPABILITIES=all \
        --volume $ROOT/data:/data \
        --entrypoint /path/to/docker_entrypoint.sh \
        your_docker_image $V4L2_DEVICES >> $LOG_FILE 2>&1
fi

# Log completion of script
log_message "d2d_service_docker_run.sh execution completed"