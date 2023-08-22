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

print_info "Building mzahana/d2dtracker-jetson:r${L4T_VERSION} ..."
cd $ROOT/docker && make d2dtracker-jetson L4TVER=${L4T_VERSION} UNAME="${USERNAME}" USER_ID=`id -u` U_GID=`id -g`

source $HOME/.bashrc

CONTAINER_NAME="d2dtracker-container"
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume" ]; then
    print_info "Creating container's shared volume: $HOME/${CONTAINER_NAME}_shared_volume" && sleep 1
    mkdir $HOME/${CONTAINER_NAME}_shared_volume
fi
print_info "copying bash.sh to container shared volume at ${HOME}/${CONTAINER_NAME}_shared_volume" && sleep 1
cp $ROOT/scripts/bash.sh $HOME/${CONTAINER_NAME}_shared_volume/

echo "You can execute " && print_info "d2dtracker_container " && echo "to start the d2dtracker-container"

print_info "Installing Arducam drivers..." && sleep 1
print_warning "Reboot your device after this step" && sleep 2

source $ROOT/scripts/arducam_drivers.sh

print_info "DONE!"