#!/bin/bash -e

USERNAME=d2d

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

sudo apt-get update && sudo apt-get install -y git python3-pip
if [ ! -d "${HOME}/jetson-containers" ]; then
    print_info "Cloning jetson-containers repo ..."
    cd ${HOME} && git clone --depth=1 https://github.com/dusty-nv/jetson-containers
else
    cd ${HOME}/jetson-containers && git pull origin master
fi
print_info "Installing requirements of jetson-containers ..."
sleep 1
cd ${HOME}/jetson-containers
pip3 install -r requirements.txt


#
# Setup the udev rules of Realsense camera
#
if [ ! -d "$HOME/src" ]; then
    mkdir -p $HOME/src
fi
echo "Setting up Realsense udev rules ..." && sleep 1
if [ ! -d "$HOME/src/librealsense" ]; then
    cd $HOME/src/
    git clone https://github.com/IntelRealSense/librealsense 
else
    cd $HOME/src/librealsense
    git pull origin master
fi
cd $HOME/src/librealsense && \
./scripts/setup_udev_rules.sh

bashrc_file="$HOME/.bashrc"
line_to_check="alias d2dtracker_container='. $ROOT/scripts/run_d2dtracker.sh'"

if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
    print_info "d2dtracker_container alias added to .bashrc file."
else
    print_warning "d2dtracker_container alias already exists in .bashrc file. No changes made."
fi

# Define the image name and tag
IMAGE_NAME="mzahana/d2dtracker-jetson"
TAG="r${L4T_VERSION}"
FULL_IMAGE_NAME="$IMAGE_NAME:$TAG"

# Check if the image already exists locally
#docker images | grep "$IMAGE_NAME" | grep "$TAG" > /dev/null 2>&1

if [[ "$FORCE_BUILD" == "true" ]]; then
	print_info "FORCE_BUILD: Building ros2_humble+pytorch+torchvision docker layer"
    cd ${HOME}/jetson-containers
    ./build.sh --name=ros_humble_pytorch pytorch torchvision ros:humble-desktop
    print_info "FORCE_BUILD: Building mzahana/d2dtracker-jetson:r${L4T_VERSION} ..."
	cd $ROOT/docker && make d2dtracker-jetson L4TVER=${L4T_VERSION} UNAME="${USERNAME}" USER_ID=`id -u` U_GID=`id -g`
else
	# $? is a special variable that holds the exit status of the last command executed
	if docker images | grep "$IMAGE_NAME" | grep "$TAG" > /dev/null 2>&1; then
	  print_info "Image $FULL_IMAGE_NAME already exists locally."
	else
	    # Try to pull the image
	    print_info "Trying to pull $FULL_IMAGE_NAME"
	    #docker pull $FULL_IMAGE_NAME

	    # Check if the pull was successful
	    if docker pull $FULL_IMAGE_NAME; then
		    print_info "Successfully pulled $FULL_IMAGE_NAME"
	    else
            print_error "Failed to pull $FULL_IMAGE_NAME, building locally..."
            
            print_info "Building ros2_humble+pytorch+torchvision docker layer"
            cd ${HOME}/jetson-containers
            ./build.sh --name=ros_humble_pytorch pytorch torchvision ros:humble-desktop

            print_info "Building mzahana/d2dtracker-jetson:r${L4T_VERSION} ..."
            cd $ROOT/docker && make d2dtracker-jetson L4TVER=${L4T_VERSION} UNAME="${USERNAME}" USER_ID=`id -u` U_GID=`id -g`
	    fi
	fi
fi

source $HOME/.bashrc

CONTAINER_NAME="d2dtracker-container"
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume" ]; then
    print_info "Creating container's shared volume: $HOME/${CONTAINER_NAME}_shared_volume" && sleep 1
    mkdir $HOME/${CONTAINER_NAME}_shared_volume
fi
print_info "copying config.sh to container shared volume at ${HOME}/${CONTAINER_NAME}_shared_volume" && sleep 1
cp $ROOT/scripts/config.sh $HOME/${CONTAINER_NAME}_shared_volume/


if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws" ]; then
    print_info "Creating ros2_ws in $HOME/${CONTAINER_NAME}_shared_volume" && sleep 1
    mkdir $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws
fi
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src" ]; then
    mkdir $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
fi

#
# Cloning d2dtracker_system
#
print_info "Cloning d2dtracker_system package into $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src"
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/d2dtracker_system" ]; then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone https://github.com/mzahana/d2dtracker_system.git
else
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/d2dtracker_system
    git pull origin main
fi

#
# Cloning open_vins
#
print_info "Cloning open_vins ..." && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/open_vins" ];then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone https://github.com/rpng/open_vins/
else
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/open_vins
    git pull origin master
fi
print_info "patching ROS2Visualizer.h ..." && sleep 1
cp $ROOT/docker/patches/ROS2Visualizer.h $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/open_vins/ov_msckf/src/ros/
####################### Done cloneing open_vins #####################

#
# Clone realsense-ros 4.51.1
#
print_info "Cloning realsense-ros ... " && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/realsense-ros" ]; then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone https://github.com/IntelRealSense/realsense-ros.git -b 4.51.1
else
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/realsense-ros
    git checkout 4.51.1
fi
####################### Done with realsense-ros #####################

#
# mavlink-router
#
#deps
# sudo apt install -y git meson ninja-build pkg-config gcc g++ systemd python3-pip
# sudo pip3 install --upgrade meson
# cd $HOME
# print_info "installing mavlink-router ... " && sleep 1
# if [ ! -d "$HOME/mavlink-router" ]; then
#     cd $HOME
#     git clone https://github.com/mavlink-router/mavlink-router.git
# else
#     cd $HOME/mavlink-router
#     git pull origin master
# fi
# cd $HOME/mavlink-router
# git submodule update --init --recursive
# meson setup --buildtype=release build .
# ninja -C build
# sudo ninja -C build install
# print_info "Copying custom mavlink-router.service to /lib/systemd/system" && sleep 1
# sudo cp $ROOT/services/mavlink-router.service /lib/systemd/system/
# if [ ! -d "/etc/mavlink-router" ]; then
#     cd $HOME
#     sudo mkdir -p /etc/mavlink-router
# fi
# sudo cp $ROOT/config/mavlink-router/main.conf /etc/mavlink-router/
# bashrc_file="$HOME/.bashrc"
# line_to_check="export MAVLINK_ROUTERD_CONF_FILE=$HOME/d2dtracker_jetson/config/mavlink-router/main.conf"
# if ! grep -qF "$line_to_check" "$bashrc_file"; then
#     echo "$line_to_check" >> "$bashrc_file"
#     print_info "MAVLINK_ROUTERD_CONF_FILE is added to .bashrc file."
# else
#     print_warning "MAVLINK_ROUTERD_CONF_FILE already exists in .bashrc file. No changes made."
# fi
# sudo systemctl daemon-reload
# print_info "You can start mavlink-router using: sudo systemctl start mavlink-router.service" && sleep 1
####################### Done with mavlink-router #####################

#
# mavros dependencies
#
# Some depndencies are installed in the docker image Dockerfile.d2dtracker
print_info "Cloning geographic_info  package ... " && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/geographic_info" ]; then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone -b 1.0.5 https://github.com/ros-geographic-info/geographic_info
fi
# print_info "Cloning angles package ... " && sleep 1
# if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/angles" ]; then
#     cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
#     git clone -b 1.15.0 https://github.com/ros/angles.git
# fi
print_info "Cloning eigen_stl_containers package ... " && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/eigen_stl_containers" ]; then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone -b ros2 https://github.com/ros/eigen_stl_containers.git
fi

# these mavlink and mavros versions are working for ros2 humble
# Sept 17, 2023
print_info "Cloning mavlink package ... " && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/mavlink" ]; then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone  https://github.com/ros2-gbp/mavlink-gbp-release.git mavlink
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/mavlink && git checkout release/humble/mavlink/2023.9.9-1
fi
print_info "Cloning mavros package ... " && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/mavros" ]; then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone  https://github.com/mavlink/mavros.git
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/mavros && git checkout 2.6.0
fi
####################### Done with mavros #####################

#
# topic_tools
#
print_info "Cloning topic_tools package ... " && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/topic_tools" ]; then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone -b humble https://github.com/ros-tooling/topic_tools.git
fi

####################### Done with topic_tools #####################

#
# apriltag
#
print_info "Cloning apriltag package ... " && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/apriltag" ]; then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone https://github.com/AprilRobotics/apriltag.git
fi

#
# apriltag_ros
#
print_info "Cloning apriltag_ros package ... " && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/apriltag_ros" ]; then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone https://github.com/mzahana/apriltag_ros.git
fi

#
# apriltag_tools_ros
#
print_info "Cloning apriltag_tools_ros package ... " && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/apriltag_tools_ros" ]; then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone https://github.com/khaledgabr77/apriltag_tools_ros.git
else
    print_info "Pulling latest apriltag_tools_ros..."
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/apriltag_tools_ros && git pull origin main
fi

#
# diagnostics , required by realsense_camera ros pkg
#
print_info "Cloning diagnostics package ... " && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/diagnostics" ]; then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone -b ros2 https://github.com/ros/diagnostics.git
fi

#
# rqt_tf_tree 
#
print_info "Cloning rqt_tf_tree package ... " && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/rqt_tf_tree" ]; then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone -b humble https://github.com/ros-visualization/rqt_tf_tree.git
fi

#
# install d2dtracker.service
#
print_info "Copying d2dtracker.service to /etc/systemd/system/ " && sleep 1
sudo cp $ROOT/services/d2dtracker.service /etc/systemd/system/
print_info "Copying d2d_service_entrypoint.sh to $HOME/${CONTAINER_NAME}_shared_volume/ " && sleep 1
cp $ROOT/services/d2d_service_entrypoint.sh $HOME/${CONTAINER_NAME}_shared_volume/
sudo systemctl daemon-reload
print_info "Enable d2dtracker.service using: sudo systemctl enable d2dtracker.service"
print_info "Start d2dtracker.service using: sudo systemctl start d2dtracker.service" && sleep 1
######################## Done with copying d2dtracker.service ##############

#
# Copy rtps_udp_profile.xml
#
print_info "Copying $ROOT/docker/middleware_profiles/rtps_udp_profile.xml to $HOME/${CONTAINER_NAME}_shared_volume/ " && sleep 1
cp $ROOT/docker/middleware_profiles/rtps_udp_profile.xml $HOME/${CONTAINER_NAME}_shared_volume/
###########################

#
# Add udev rules for /dev/ttyUSB0 so MAVROS can use it
#
sudo cp $ROOT/docker/udev_rules/99-usb-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
######################33 Done with udev rules ###########
#
# Arducam drivers
# This is to use Arducam camera array adapter
# Example product: 
#   https://www.uctronics.com/arducam-1mp-4-quadrascopic-camera-bundle-kit-for-raspberry-pi-nvidia-jetson-nano-xavier-nx.html
#
print_info "Installing Arducam drivers..." && sleep 1
print_warning "Reboot your device after this step" && sleep 2

cd ${HOME}
sudo apt-get install v4l-utils -y

wget https://bootstrap.pypa.io/get-pip.py  
sudo python3 get-pip.py  
sudo pip3 install v4l2-fix

# Install Jtop
sudo pip3 install -U jetson-stats

# REF: https://docs.arducam.com/Nvidia-Jetson-Camera/Jetvariety-Camera/Quick-Start-Guide/
cd ${HOIME}
wget https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/install_full.sh
chmod +x install_full.sh
./install_full.sh -m arducam

# source $ROOT/scripts/arducam_drivers.sh

echo "You can execute " && print_info "d2dtracker_container " && echo "to start the d2dtracker-container"
print_warning "If this is the first time you setup d2dtracker on Jetson, enter the container and run the setup.sh script inside the d2dtracker_system pkg, inside the container"

print_info "DONE!"
cd $HOME
