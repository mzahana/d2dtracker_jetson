#!/bin/bash -e

sudo cp scripts/daemon.json /etc/docker/daemon.json
echo && echo "daemon.json is copied to /etc/docker/daemon.json" && echo
sleep 1

echo "Reloading docker ..."
sleep 1
sudo systemctl daemon-reload && sudo systemctl restart docker

echo "Install Git LFS in order to pull down all large files..."
sleep 1
sudo apt-get install git-lfs -y
git lfs install --skip-repo

echo "Create ROS workspace at $HOME/workspaces "
sleep 1

if [ ! -d "$HOME/workspaces" ]; then
    mkdir -p $HOME/workspaces/isaac_ros-dev/src
fi

echo "Cloning Nvidia ROS packages ..."
sleep 1

#
# isaac_ros_visual_slam
#
if [ ! -d "$HOME/workspaces/isaac_ros-dev/src/isaac_ros_visual_slam" ]; then
    cd $HOME/workspaces/isaac_ros-dev/src
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
else
    echo "Pulling latest isaac_ros_visual_slam..." && sleep 1
    cd $HOME/workspaces/isaac_ros-dev/src/isaac_ros_visual_slam
    git pull origin main
fi

#
# isaac_ros_common
#
if [ ! -d "$HOME/workspaces/isaac_ros-dev/src/isaac_ros_common" ]; then
    cd $HOME/workspaces/isaac_ros-dev/src
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
else
    echo "Pulling latest isaac_ros_common..." && sleep 1
    cd $HOME/workspaces/isaac_ros-dev/src/isaac_ros_common
    git pull origin main
fi

#
# isaac_ros_nitros
#
if [ ! -d "$HOME/workspaces/isaac_ros-dev/src/isaac_ros_nitros" ]; then
    cd $HOME/workspaces/isaac_ros-dev/src
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
else
    echo "Pulling latest isaac_ros_nitros..." && sleep 1
    cd $HOME/workspaces/isaac_ros-dev/src/isaac_ros_nitros
    git pull origin main
fi

#
# Setup the udev rules of Realsense camera
echo "Setting up Realsense udev rules ..." && sleep 1
cd /tmp && \
git clone https://github.com/IntelRealSense/librealsense && \
cd librealsense && \
./scripts/setup_udev_rules.sh

#
# Clone realsense-ros 4.51.1
#
echo "Cloning realsense-ros ... " && sleep 1
if [ ! -d "$HOME/workspaces/isaac_ros-dev/src/realsense-ros" ]; then
    git clone https://github.com/IntelRealSense/realsense-ros.git -b 4.51.1
else
    cd $HOME/workspaces/isaac_ros-dev/src/realsense-ros
    git checkout 4.51.1
fi

echo "Confgiuring container..." && sleep 1
cd $HOME/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts && \
rm .isaac_ros_common-config && \
touch .isaac_ros_common-config && \
echo CONFIG_IMAGE_KEY=ros2_humble.realsense > .isaac_ros_common-config

# Copy custom Dockerfile for MicroAgent and d2dtracker pkgs
# Set the $CONFIG_IMAGE_KEY in isaac_ros_common/scripts/.isaac_ros_common-config
# Copy the modified run_dev.sh file