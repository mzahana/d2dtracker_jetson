# d2dtracker_jetson
This package can be used to set up the D2DTracker system on Nvidia Jetson paltforms. 

# Quick Setup
**NOTE: All th following steps should be done on the Jetson board.**

* Clone this repository 
    ```BASH
    cd $HOME
    git clone https://github.com/mzahana/d2dtracker_jetson.git
    ```
* Export `GIT_USER` and `GIT_TOKEN` environment variables (contact admin)
    ```bash
    export GIT_USER=github_user_ID
    export GIT_TOKEN=token
    ```
* Enter the repo's directory and execute the `setup.sh` script
```bash
cd $HOME/d2dtracker_jetson
./setup.sh
```
**NOTE: Provide `sudo` password when asked.**

# Run System
* After completing the Quick Setup step, run the docker container
```bash
source $HOME/.bashrc
d2dtracker_container
```
* `d2dtracker_container` is an alias that is created by the `setup.sh` script and written to `$HOME/.bashrc` to easily access the container.

# Setup (to be removed)
* It is HIGHLY recommended to install an SSD on your Jetson device.
* Physically install the NVMe SSD on a fresh Jetson developer kit, then use SDK Manager running on an Ubuntu PC to flash the entire L4T (Jetson Linux) on the SSD
* Configure `nvidia-container-runtime` as the default runtime for Docker.
    
    Using your text editor of choice, add the following items to `/etc/docker/daemon.json`.
    ```bash
    {
        "runtimes": {
            "nvidia": {
                "path": "nvidia-container-runtime",
                "runtimeArgs": []
            }
        },

        "default-runtime": "nvidia"
    }
    ```
* Restart docker
    ```bash
    sudo systemctl daemon-reload && sudo systemctl restart docker
    ```
* Install Git LFS in order to pull down all large files:
    ```bash
    sudo apt-get install git-lfs
    git lfs install --skip-repo
    ```
* Finally, create a ROS 2 workspace for experimenting with ROS packages
    ```bash
    mkdir -p  ~/workspaces/isaac_ros-dev/src
    echo "export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev/" >> ~/.bashrc
    source ~/.bashrc
    ```
* Clone the ISAAC ROS packages to the workspace
    ```bash
    cd ${ISAAC_ROS_WS}/src
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
    ```