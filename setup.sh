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

#
# Setup the udev rules of Realsense camera
#
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

bashrc_file="$HOME/.bashrc"
line_to_check="alias d2dtracker='. $ROOT/scripts/run_d2dtracker2.sh'"

if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
    print_info "d2dtracker alias added to .bashrc file."
else
    print_warning "d2dtracker alias already exists in .bashrc file. No changes made."
fi

print_info "Building mzahana/d2dtracker-jetson:r${L4T_VERSION} ..."
cd $ROOT/docker && make d2dtracker-jetson

source $HOME/.bashrc

echo "Execute " && print_info "d2dtracker " && echo "to start the d2dtracker-container2"

print_info "DONE!"