#!/bin/bash -e

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/utils/print_color.sh
source $ROOT/utils/l4t_version.sh

sudo cp $ROOT/scripts/daemon.json /etc/docker/daemon.json
print_info "daemon.json is copied to /etc/docker/daemon.json"
sleep 1

print_info "Reloading docker ..."
sleep 1
sudo systemctl daemon-reload && sudo systemctl restart docker

print_info "Install Git LFS in order to pull down all large files..."
sleep 1
sudo apt-get install git-lfs -y
git lfs install --skip-repo


bashrc_file="$HOME/.bashrc"
line_to_check="export ISAAC_ROS_WS=\${HOME}/workspaces/isaac_ros-dev/"

if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
    print_info "ISAAC_ROS_WS is exported to .bashrc file."
else
    print_warning "ISAAC_ROS_WS is already exported .bashrc file. No changes made."
fi

export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev
export WORKSPACES_PATH=${HOME}/workspaces

echo "Create ROS workspace at $HOME/workspaces "
sleep 1
if [ ! -d "$HOME/workspaces" ]; then
    print_info "Creating $HOME/workspaces/isaac_ros-dev/src" && sleep 1
    mkdir -p $HOME/workspaces/isaac_ros-dev/src
fi

if [ ! -d "$HOME/workspaces/isaac_ros-dev" ]; then
    print_info "Creating $HOME/workspaces/isaac_ros-dev/src" && sleep 1
    mkdir -p $HOME/workspaces/isaac_ros-dev/src
fi

print_info "copying config.sh to container shared volume at ${HOME}/workspaces/" && sleep 1
cp $ROOT/scripts/config.sh ${HOME}/workspaces/

print_info "Cloning Nvidia ROS packages ..."
sleep 1

#
# isaac_ros_visual_slam
#
if [ ! -d "$ISAAC_ROS_WS/src/isaac_ros_visual_slam" ]; then
    cd $ISAAC_ROS_WS/src
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
else
    print_info "Pulling latest isaac_ros_visual_slam..." && sleep 1
    cd $ISAAC_ROS_WS/src/isaac_ros_visual_slam
    git pull origin main
fi

#
# isaac_ros_common
#
if [ ! -d "$ISAAC_ROS_WS/src/isaac_ros_common" ]; then
    cd $ISAAC_ROS_WS/src
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
else
    print_info "Pulling latest isaac_ros_common..." && sleep 1
    cd $ISAAC_ROS_WS/src/isaac_ros_common
    git pull origin main
fi

#
# isaac_ros_nitros
#
if [ ! -d "$ISAAC_ROS_WS/src/isaac_ros_nitros" ]; then
    cd $ISAAC_ROS_WS/src
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
else
    print_info "Pulling latest isaac_ros_nitros..." && sleep 1
    cd $ISAAC_ROS_WS/src/isaac_ros_nitros
    git pull origin main
fi

#
# isaac_ros_object_detection
#
if [ ! -d "$ISAAC_ROS_WS/src/isaac_ros_object_detection" ]; then
    cd $ISAAC_ROS_WS/src
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git
else
    print_info "Pulling latest isaac_ros_object_detection..." && sleep 1
    cd $ISAAC_ROS_WS/src/isaac_ros_object_detection
    git pull origin main
fi

#
# isaac_ros_dnn_stereo_depth
#
if [ ! -d "$ISAAC_ROS_WS/src/isaac_ros_dnn_stereo_depth" ]; then
    cd $ISAAC_ROS_WS/src
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth.git
else
    print_info "Pulling latest isaac_ros_dnn_stereo_depth..." && sleep 1
    cd $ISAAC_ROS_WS/src/isaac_ros_dnn_stereo_depth
    git pull origin main
fi

#
# isaac_ros_system
#
if [ ! -d "$ISAAC_ROS_WS/src/isaac_ros_system" ]; then
    cd $ISAAC_ROS_WS/src
    git clone https://github.com/mzahana/isaac_ros_system.git
else
    print_info "Pulling latest isaac_ros_system..." && sleep 1
    cd $ISAAC_ROS_WS/src/isaac_ros_system
    git pull origin main
fi

#
# Setup the udev rules of Realsense camera
if [ ! -d "$HOME/src" ]; then
    mkdir $HOME/src
fi
echo "Setting up Realsense udev rules ..." && sleep 1
if [ ! -d "$HOME/src/librealsense" ]; then
    cd $HOME/src
    git clone https://github.com/IntelRealSense/librealsense 
else
    cd $HOME/src/librealsense
    git pull origin master
fi
cd $HOME/src/librealsense && \
./scripts/setup_udev_rules.sh

#
# Clone realsense-ros 4.51.1
#
print_info "Cloning realsense-ros ... " && sleep 1
if [ ! -d "$ISAAC_ROS_WS/src/realsense-ros" ]; then
    cd $ISAAC_ROS_WS/src
    git clone https://github.com/IntelRealSense/realsense-ros.git -b 4.51.1
else
    cd $ISAAC_ROS_WS/src/realsense-ros
    git checkout 4.51.1
fi

print_info "Configuring container..." && sleep 1
if [ -f "$ISAAC_ROS_WS/src/isaac_ros_common/scripts/.isaac_ros_common-config" ]; then
    cd $ISAAC_ROS_WS/src/isaac_ros_common/scripts
    rm .isaac_ros_common-config
fi
cd $ISAAC_ROS_WS/src/isaac_ros_common/scripts
touch .isaac_ros_common-config && \
echo CONFIG_IMAGE_KEY=ros2_humble.realsense > .isaac_ros_common-config

# run_nvidia_container.sh
print_info "Copying run_nvidia_container.sh to $ISAAC_ROS_WS/src/isaac_ros_common/scripts " && sleep 1
cp $ROOT/scripts/run_nvidia_container.sh $ISAAC_ROS_WS/src/isaac_ros_common/scripts/

# Dockerfile.realsense
print_info "Copying Dockerfile.realsense to $ISAAC_ROS_WS/src/isaac_ros_common/docker" && sleep 1
cp $ROOT/docker/Dockerfile.realsense $ISAAC_ROS_WS/src/isaac_ros_common/docker/

# Dockerfile.user
print_info "Copying Dockerfile.user to $ISAAC_ROS_WS/src/isaac_ros_common/docker" && sleep 1
cp $ROOT/docker/Dockerfile.user $ISAAC_ROS_WS/src/isaac_ros_common/docker/

# modified-workspace-entrypoint.sh 
print_info "Copying modified-workspace-entrypoint.sh to $ISAAC_ROS_WS/src/isaac_ros_common/docker/scripts" && sleep 1
cp $ROOT/scripts/modified-workspace-entrypoint.sh $ISAAC_ROS_WS/src/isaac_ros_common/docker/scripts/

# build_nvidia_image.sh
print_info "Copying build_nvidia_image.sh to $ISAAC_ROS_WS/src/isaac_ros_common/scripts" && sleep 1
cp $ROOT/scripts/build_nvidia_image.sh $ISAAC_ROS_WS/src/isaac_ros_common/scripts/

# print_info "Copying config.sh to ~/workspaces/" && sleep 1
# cp $ROOT/scripts/config.sh $WORKSPACES_PATH/

print_info "Copying vslam_realsense.launch.py to $ISAAC_ROS_WS/src/isaac_ros_visual_slam/isaac_ros_visual_slam/launch" && sleep 1
cp $ROOT/launch/vslam_realsense.launch.py $ISAAC_ROS_WS/src/isaac_ros_visual_slam/isaac_ros_visual_slam/launch


bashrc_file="$HOME/.bashrc"
line_to_check="alias isaac_ros_container='. $ISAAC_ROS_WS/src/isaac_ros_common/scripts/run_dev.sh'"

if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
    print_info "isaac_ros_container alias added to .bashrc file."
else
    print_warning "isaac_ros_container alias already exists in .bashrc file. No changes made."
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias nvidia_container='. $ISAAC_ROS_WS/src/isaac_ros_common/scripts/run_nvidia_container.sh'"

if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
    print_info "nvidia_container alias added to .bashrc file."
else
    print_warning "nvidia_container alias already exists in .bashrc file. No changes made."
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias build_nvidia_image='. $ISAAC_ROS_WS/src/isaac_ros_common/scripts/build_nvidia_image.sh'"

if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
    print_info "build_nvidia_image alias added to .bashrc file."
else
    print_warning "build_nvidia_image alias already exists in .bashrc file. No changes made."
fi

# Define the image name and tag
IMAGE_NAME="mzahana/isaac_ros_dev-aarch64"
TAG="${L4T_VERSION}"
FULL_IMAGE_NAME="$IMAGE_NAME:$TAG"

print_info "Looking for $FULL_IMAGE_NAME"

# Check if the image already exists locally
if docker images | grep "$IMAGE_NAME" | grep "$TAG" > /dev/null 2>&1; then
    echo "Image $FULL_IMAGE_NAME already exists locally."
else
    # Try to pull the image
    print_info "Trying to pull $FULL_IMAGE_NAME"

    # Check if the pull was successful
    if docker pull $FULL_IMAGE_NAME; then
        print_info "Successfully pulled $FULL_IMAGE_NAME"
    else
        print_error "Failed to pull $FULL_IMAGE_NAME, building locally..."
        print_info "Building nvidia-image ..."
        cd $ISAAC_ROS_WS/src/isaac_ros_common/scripts && ./build_nvidia_image.sh
    fi
fi

# Need to re-tag the image from "mzahana/isaac_ros_dev-aarch64"  to "isaac_ros_dev-aarch64"
# Check if the image isaac_ros_dev-aarch64:latest exists locally
latest_exists=$(docker images -q isaac_ros_dev-aarch64:latest)

# If the image isaac_ros_dev-aarch64:latest does not exist, retag the image
if [ -z "$latest_exists" ]; then
    print_info "Image isaac_ros_dev-aarch64:latest does not exist. Retagging..."
    docker tag mzahana/isaac_ros_dev-aarch64:${TAG} isaac_ros_dev-aarch64:latest
else
    print_warning "Image isaac_ros_dev-aarch64:latest already exists. Doing nothing."
fi

source $HOME/.bashrc

echo "Execute " && print_info "nvidia_container " && echo "to start the nvidia-container"

print_info "DONE!"
