#!/usr/bin/env bash
# pass-through commands to 'docker run' with some defaults
# https://docs.docker.com/engine/reference/commandline/run/
ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# check for V4L2 devices
V4L2_DEVICES=""

for i in {0..9}
do
	if [ -a "/dev/video$i" ]; then
		V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
	fi
done

# check for display
DISPLAY_DEVICE=""

if [ -n "$DISPLAY" ]; then
	# give docker root user X11 permissions
	sudo xhost +si:localuser:root
	
	# enable SSH X11 forwarding inside container (https://stackoverflow.com/q/48235040)
	XAUTH=/tmp/.docker.xauth
	xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
	chmod 777 $XAUTH

	DISPLAY_DEVICE="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH"
fi

# check if sudo is needed
if id -nG "$USER" | grep -qw "docker"; then
	SUDO=""
else
	SUDO="sudo"
fi

# Restart if already running
## Default CMD if none provided
DEFAULT_CMD="/bin/bash"
CONTAINER_NAME="d2dtracker-container"
HOST_USERNAME=hunter2
DOCKER_USERNAME=d2d

# Use the first command line argument as CMD, fallback to DEFAULT_CMD if not provided
CMD="${1:-$DEFAULT_CMD}"

# Check if the container exists and is running
if [ "$($SUDO docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    # Check if the container is exited and needs to be restarted
    if [ "$($SUDO docker ps -aq -f status=exited -f name=${CONTAINER_NAME})" ]; then
        echo "Restarting the container..."
        $SUDO docker start ${CONTAINER_NAME}
    fi
    # Execute the provided command inside the container, passing the DISPLAY variable and mounting the X11 socket
    $SUDO docker exec -it \
      -e DISPLAY=$DISPLAY \
      --workdir /home/$DOCKER_USERNAME/shared_volume \
      ${CONTAINER_NAME} \
      env TERM=xterm-256color bash -c "${CMD}"
    exit 0
else
    echo "$CONTAINER_NAME is not running. You may execute d2dtracker_container to run it."
fi