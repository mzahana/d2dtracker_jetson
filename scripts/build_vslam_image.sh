#!/bin/bash
#
# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/utils/print_color.sh

function usage() {
    print_info "Usage: run_dev.sh" {isaac_ros_dev directory path OPTIONAL}
    print_info "Copyright (c) 2021-2022, NVIDIA CORPORATION."
}

# Read and parse config file if exists
#
# CONFIG_IMAGE_KEY (string, can be empty)

if [[ -f "${ROOT}/.isaac_ros_common-config" ]]; then
    . "${ROOT}/.isaac_ros_common-config"
fi

ISAAC_ROS_DEV_DIR="$1"
if [[ -z "$ISAAC_ROS_DEV_DIR" ]]; then
    ISAAC_ROS_DEV_DIR="$HOME/workspaces/isaac_ros-dev"
    if [[ ! -d "$ISAAC_ROS_DEV_DIR" ]]; then
        ISAAC_ROS_DEV_DIR=$(pwd)
    fi
    print_warning "isaac_ros_dev not specified, assuming $ISAAC_ROS_DEV_DIR"
else
    shift 1
fi

ON_EXIT=()
function cleanup {
    for command in "${ON_EXIT[@]}"
    do
        $command
    done
}
trap cleanup EXIT

pushd . >/dev/null
cd $ROOT
ON_EXIT+=("popd")

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

# Check if git-lfs is installed.
if [[ -z "$(git lfs)" ]] ; then
    print_error "git-lfs is not insalled. Please make sure git-lfs is installed before you clone the repo."
    exit 1
fi

# Check if all LFS files are in place
git rev-parse &>/dev/null
if [[ $? -eq 0 ]]; then
    LFS_FILES_STATUS=$(cd $ISAAC_ROS_DEV_DIR && git lfs ls-files | cut -d ' ' -f2)
    for (( i=0; i<${#LFS_FILES_STATUS}; i++ )); do
        f="${LFS_FILES_STATUS:$i:1}"
        if [[ "$f" == "-" ]]; then
            print_error "LFS files are missing. Please re-clone the repo after installing git-lfs."
            exit 1
        fi
    done
fi

PLATFORM="$(uname -m)"

BASE_NAME="isaac_ros_dev-$PLATFORM"
# CONTAINER_NAME="$BASE_NAME-container"
CONTAINER_NAME="vslam-container"

# Remove any exited containers.
if [ "$(docker ps -a --quiet --filter status=exited --filter name=$CONTAINER_NAME)" ]; then
    docker rm $CONTAINER_NAME > /dev/null
fi

# Build image
IMAGE_KEY=ros2_humble
if [[ ! -z "${CONFIG_IMAGE_KEY}" ]]; then
    IMAGE_KEY=$CONFIG_IMAGE_KEY
fi

BASE_IMAGE_KEY=$PLATFORM.user
if [[ ! -z "${IMAGE_KEY}" ]]; then
    BASE_IMAGE_KEY=$PLATFORM.$IMAGE_KEY

    # If the configured key does not have .user, append it last
    if [[ $IMAGE_KEY != *".user"* ]]; then
        BASE_IMAGE_KEY=$BASE_IMAGE_KEY.user
    fi
fi

print_info "Building $BASE_IMAGE_KEY base as image: $BASE_NAME using key $BASE_IMAGE_KEY"
$ROOT/build_base_image.sh $BASE_IMAGE_KEY $BASE_NAME '' '' ''

if [ $? -ne 0 ]; then
    print_error "Failed to build base image: $BASE_NAME, aborting."
    exit 1
fi
