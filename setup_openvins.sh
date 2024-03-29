#!/bin/bash -e

USERNAME=vins

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/utils/print_color.sh
source $ROOT/utils/l4t_version.sh

sudo cp $ROOT/scripts/daemon.json /etc/docker/daemon.json
print_info "daemon.json is copied to /etc/docker/daemon.json"
sleep 1

print_info "Reloading docker ..."
sleep 1
sudo systemctl daemon-reload && sudo systemctl restart docker

CONTAINER_NAME="openvins-container"
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume" ]; then
    print_info "Creating container's shared volume: $HOME/${CONTAINER_NAME}_shared_volume" && sleep 1
    mkdir -p $HOME/${CONTAINER_NAME}_shared_volume
fi

print_info "copying bash.sh to container shared volume at ${HOME}/${CONTAINER_NAME}_shared_volume" && sleep 1
cp $ROOT/scripts/bash.sh $HOME/${CONTAINER_NAME}_shared_volume/

if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws" ];then
    mkdir -p $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws
fi
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src" ];then
    mkdir -p $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
fi

print_info "Cloning openvins ..." && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/open_vins" ];then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone https://github.com/rpng/open_vins/
else
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/open_vins
    git pull origin master
fi
print_info "patching ROS2Visualizer.h ..." && sleep 1
cp $ROOT/docker/patches/ROS2Visualizer.h $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/open_vins/ov_msckf/src/ros/

bashrc_file="$HOME/.bashrc"
line_to_check="alias openvins_container='. $ROOT/scripts/run_openvins.sh'"

if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
    print_info "openvins_container alias added to .bashrc file."
else
    print_warning "openvins_container alias already exists in .bashrc file. No changes made."
fi

# Define the image name and tag
IMAGE_NAME="mzahana/openvins-jetson"
TAG="${L4T_VERSION}"
FULL_IMAGE_NAME="$IMAGE_NAME:$TAG"

# Check if the image already exists locally
docker images | grep "$IMAGE_NAME" | grep "$TAG" > /dev/null 2>&1

# $? is a special variable that holds the exit status of the last command executed
if [ $? -eq 0 ]; then
  echo "Image $FULL_IMAGE_NAME already exists locally."
else
    # Try to pull the image
    print_info "Trying to pull $FULL_IMAGE_NAME"
    docker pull $FULL_IMAGE_NAME

    # Check if the pull was successful
    if [ $? -eq 0 ]; then
        print_info "Successfully pulled $FULL_IMAGE_NAME"
    else
        print_error "Failed to pull $FULL_IMAGE_NAME, building locally..."
        print_info "Building mzahana/openvins-jetson:r${L4T_VERSION} ..."
        cd $ROOT/docker && make openvins-jetson L4TVER=${L4T_VERSION} UNAME="${USERNAME}" USER_ID=`id -u` U_GID=`id -g`
    fi
fi

source $HOME/.bashrc


echo "You can execute " && print_info "openvins_container " && echo "to start the openvins-container"


print_info "DONE!"