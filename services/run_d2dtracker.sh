#!/bin/bash
#
# Copyright (c) 2023, Mohamed Abdelkader.  All rights reserved.

USERNAME=d2d
RUN_SYSTEM=True




PLATFORM="$(uname -m)"

BASE_NAME="mzahana/d2dtracker-jetson:r${L4T_VERSION}"
CONTAINER_NAME="d2dtracker-container"


CMD="export DEV_DIR=\$HOME/shared_volume && \
        if [[ -f "\$HOME/shared_volume/ros2_ws/install/setup.bash" ]]; then
            source \$HOME/shared_volume/ros2_ws/install/setup.bash
        fi && \
        if [[ -f "\$HOME/shared_volume/bash.sh" ]]; then
            source \$HOME/shared_volume/bash.sh
        fi"
if [[ -n "$GIT_TOKEN" ]] && [[ -n "$GIT_USER" ]]; then
    CMD="export GIT_USER=$GIT_USER && export GIT_TOKEN=$GIT_TOKEN && $CMD"
fi


if [ "$RUN_SYSTEM" == "True" ]; then
    CMD="$CMD && ros2 launch d2dtracker_system run_system.launch.py"
fi

CMD="$CMD && /bin/bash"

if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    if [ "$(docker ps -aq -f status=exited -f name=${CONTAINER_NAME})" ]; then
        # cleanup
        echo "Restarting the container..."
        docker start ${CONTAINER_NAME}
    fi
    #docker exec -it --workdir /home/${USERNAME}/shared_volume ${CONTAINER_NAME} env TERM=xterm-256color bash -c "${CMD}"
    docker exec --workdir /home/${USERNAME}/shared_volume ${CONTAINER_NAME}  bash -c "${CMD}"

fi

