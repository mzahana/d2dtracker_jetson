#!/bin/bash -e

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/utils/print_color.sh

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


echo "Create ROS workspace at $HOME/workspaces "
sleep 1
if [ ! -d "$HOME/workspaces" ]; then
    mkdir -p $HOME/workspaces/isaac_ros-dev/src
fi

if [ ! -d "$HOME/workspaces/isaac_ros-dev" ]; then
    mkdir -p $HOME/workspaces/isaac_ros-dev/src
fi

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
# Setup the udev rules of Realsense camera
echo "Setting up Realsense udev rules ..." && sleep 1
if [ ! -d "/tmp/librealsense" ]; then
    cd /tmp
    git clone https://github.com/IntelRealSense/librealsense 
else
    cd /tmp/librealsense
    git pull origin master
fi
cd /tmp/librealsense && \
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

print_info "Confgiuring container..." && sleep 1
if [ -f "$ISAAC_ROS_WS/src/isaac_ros_common/scripts/.isaac_ros_common-config" ]; then
    cd $ISAAC_ROS_WS/src/isaac_ros_common/scripts
    rm .isaac_ros_common-config
fi
cd $ISAAC_ROS_WS/src/isaac_ros_common/scripts
touch .isaac_ros_common-config && \
echo CONFIG_IMAGE_KEY=ros2_humble.realsense.d2dtracker > .isaac_ros_common-config

# run_d2dtracker.sh
print_info "Copying run_d2dtracker.sh to $ISAAC_ROS_WS/src/isaac_ros_common/scripts " && sleep 1
cp $ROOT/scripts/run_d2dtracker.sh $ISAAC_ROS_WS/src/isaac_ros_common/scripts/

# Dockerfile.d2dtracker
print_info "Copying Dockerfile.d2dtracker to $ISAAC_ROS_WS/src/isaac_ros_common/docker" && sleep 1
cp $ROOT/docker/Dockerfile.d2dtracker $ISAAC_ROS_WS/src/isaac_ros_common/docker/

# modified-workspace-entrypoint.sh 
print_info "Copying modified-workspace-entrypoint.sh to $ISAAC_ROS_WS/src/isaac_ros_common/docker/scripts" && sleep 1
cp $ROOT/scripts/modified-workspace-entrypoint.sh $ISAAC_ROS_WS/src/isaac_ros_common/docker/scripts/

# build_d2dtracker_image.sh
print_info "Copying build_d2dtracker_image.sh to $ISAAC_ROS_WS/src/isaac_ros_common/scripts" && sleep 1
cp $ROOT/scripts/build_d2dtracker_image.sh $ISAAC_ROS_WS/src/isaac_ros_common/scripts/

if [[ ! -d "$ISAAC_ROS_WS/src/isaac_ros_common/docker/libs"  ]]; then
    cd $ISAAC_ROS_WS/src/isaac_ros_common/docker && mkdir libs
fi

print_info "Copying nvToolsExt.h"
cp $ROOT/libs/nvToolsExt.h $ISAAC_ROS_WS/src/isaac_ros_common/docker/libs/
print_info "Copying libnvToolsExt.so"
cp $ROOT/libs/libnvToolsExt.so $ISAAC_ROS_WS/src/isaac_ros_common/docker/libs/

bashrc_file="$HOME/.bashrc"
line_to_check="alias isaac_ros_container='. $ISAAC_ROS_WS/src/isaac_ros_common/scripts/run_dev.sh'"

if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
    print_info "isaac_ros_container alias added to .bashrc file."
else
    print_warning "isaac_ros_container alias already exists in .bashrc file. No changes made."
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias d2dtracker_container='. $ISAAC_ROS_WS/src/isaac_ros_common/scripts/run_d2dtracker.sh'"

if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
    print_info "d2dtracker_container alias added to .bashrc file."
else
    print_warning "d2dtracker_container alias already exists in .bashrc file. No changes made."
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias build_d2dtracker_image='. $ISAAC_ROS_WS/src/isaac_ros_common/scripts/build_d2dtracker_image.sh'"

if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
    print_info "build_d2dtracker_image alias added to .bashrc file."
else
    print_warning "build_d2dtracker_image alias already exists in .bashrc file. No changes made."
fi
cd $ISAAC_ROS_WS/src/isaac_ros_common/scripts && ./build_d2dtracker_image.sh

source $HOME/.bashrc

print_info "Execute d2dtracker_container to start the d2dtracker-container"
