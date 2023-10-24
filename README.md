# d2dtracker_jetson
This package can be used to set up the D2DTracker system on Nvidia Jetson paltforms.

# Supported Jetpacks
* JetPack `5.1.1`. Uses NVIDIA `L4T 35.3.1`

# Quick Setup

**NOTE: All th following steps should be done on the Jetson board.**

* If you face issues with docker permissions after a fresh install of a jetpack, you can execute the following in a terminal on Jetson, and reboot.
    ```bash
    sudo usermod -aG docker $USER

    sudo chown root:docker /var/run/docker.sock

    rm -rf ~/.docker

    #reboot
    ```

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
## Setup the d2dtracker container
* setup
    ```bash
    cd $HOME/d2dtracker_jetson
    ./setup_d2dtracker.sh
    ```
* `source $HOME/.bashrc`

* run `d2dtracker_container` alias to enter the container

* [Inside the container] Setup the system
```bash
# Enter the container
d2dtracker_container
# Go to the d2dtracker_system pkg
cd $HOME/shared_volume/ros2_ws/src/d2dtracker_system/
./setup.sh
```
**NOTE** `open_vins` is already cloned in the workspace during the setup of the d2dtracker system, and no need to setup a separate docker image for `open_vins`.

### Update d2dtracker docker image
* Pull the latest version of this repo
	```bash
	cd $HOME/d2dtracker_jetson
	git pull origin main
	```
* Re-build d2dtracker docker image
	```bash
	export FROCE_BUILD=true
	./setup_d2dtracker.sh
	```

## Setup nvidia visual slam software (optional)
If you wish to test the `isaac_ros_visual_slam` [system](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam), you can build its independent docker image as follows.

* Enter the repo's directory and execute the `setup_vslam.sh` script
    ```bash
    cd $HOME/d2dtracker_jetson
    ./setup_vslam.sh
    ```
    **NOTE: Provide `sudo` password when asked.**

* `source $HOME/.bashrc`

* run `vslam_container` alias to enter the container
* After entering the container for the 1st time, you need to build the workspace.
    ```bash
    cd /workspaces/isaac_ros-dev
    colcon build
    ```
## Setup the openvins container(optional)

**NOTE `open_vins` can be built inside the `d2dtracker-jetson` docker image (see below)**

Follow these steps if you wish to build an independent docker image to test [open_vins package](https://github.com/rpng/open_vins).
* setup
    ```bash
    cd $HOME/d2dtracker_jetson
    ./setup_openvins.sh
    ```
* `source $HOME/.bashrc`

* run `openvins_container` alias to enter the container
* After entering the container for the 1st time, you need to build the workspace.
    ```bash
    cd /home/vins/shared_volume/ros2_ws
    MAKEFLAGS='j1 -l1' colcon build --executor sequential
    ```
    **We are using `MAKEFLAGS='j1 -l1'` as the Jetson may hang!**

# Run Systems
## d2dtracker
* Activate the ocntainer alias that was added to `~/.bashrc` after you run `setup_d2dtracker.sh`. This usually done once as it will be loaded automatically when you run a new terminal.
    ```bash
    source $HOME/.bashrc
    d2dtracker_container
    ```
* When you enter the container for the first time after a fresh setup, you will find only once package inside `~/shared_volume/ros2_ws/src`. You need to run the setup file to download the remaining packages and build them.
    ```bash
    cd ~/shared_volume/ros2_ws/src/d2dtracker_system
    ./setup.sh
    ```

* source the workspace
    ```bash
    source ~/shared_volume/ros2_ws/install/setup.bash
    ```
* Inside the `d2dtracker-container`, there is a `bash.sh` script inside `$HOME/shared_volume`. Edit which module you wish to run (set its flag to `True`), save and close it. Source the `bash.sh` script. Then, run
```bash
ros2 launch d2dtracker_system run_system.launch.py
```

## isaac_ros_visual_slam

```bash
source $HOME/.bashrc
vslam_container
```
* To use Realsense camera (e.g. D455), connect it (before you power up the board) and run
```bash
ros2 launch isaac_ros_visual_slam vslam_realsense.launch.py
```

## open_vins
```bash
source $HOME/.bashrc
openvins_container
```
* To test `open_vins` on ERUOC dataset, you need to download its ros bag, and then play it
* in another terminal inside the `openvins-container` run the openvins estimator
```bash
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
```

* **IMPROTANT**:  To use `open_vins` with live IMU/camera system, you will need to calibrate your cameras using Kalibr, calibrate your IMU using `allan_ros2` package (only works with ROS foxy, for now), calibrate both cameras and IMU using Kalibr. Then, use the calibration files to update the configuratoin files for `open_vins`. An example of configuration files for the Realsense D455 camera see the [config/open_vins/custom_d455](config/open_vins/custom_d455) See below video link for a tutorial.
* **Reference**: https://docs.openvins.com/gs-tutorial.html
* **Video** : https://youtu.be/rBT5O5TEOV4

# Automated System Startup
A service is setup to start the d2dtracker system. The service file is called `d2dtracker.service` and is available in the `services` directory in this package, [here](services/d2dtracker.service).

This service executes the file `/home/hunter/d2dtracker_jetson/services/d2d_service.sh`. You need to adjust the path according to your Jetson system setup. However, if you setup your jetson with username `hunter` and followed the setup instructions in this README, you won't need to change the path to this file.

This script `/home/hunter/d2dtracker_jetson/services/d2d_service.sh` runs a few commands and then runs another script(`/home/hunter/d2dtracker_jetson/services/run_d2dtracker.sh`) that starts(or restarts) the d2dtracker-container which itself runs the required launch file inside the container `run_system.launch.py` located in the `d2dtracker_system` pkg

For the first time only, you need to run the d2dtracker manually using the alias that was created during the setup process
```bash
d2dtracker_container
```
Once the container is run (and not removed), you can use the service to automatically startup  the system.

Enable d2dtracker.service using: 
```bash
sudo systemctl enable d2dtracker.service
```

You can disable the service using
```bash
sudo systemctl disable d2dtracker.service
```

Then you can start the service using
```bash
sudo systemctl start d2dtracker.service
```

Then you can stop the service using
```bash
sudo systemctl stop d2dtracker.service
```
