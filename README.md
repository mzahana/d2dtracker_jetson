# d2dtracker_jetson
This package can be used to set up the D2DTracker system on Nvidia Jetson paltforms. 

# Setup
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