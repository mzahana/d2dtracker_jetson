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
## Setup nvidia visual slam software
* Enter the repo's directory and execute the `setup_vslam.sh` script
    ```bash
    cd $HOME/d2dtracker_jetson
    ./setup_vslam.sh
    ```
    **NOTE: Provide `sudo` password when asked.**

* `source $HOME/.bashrc`

* run `vslam_container` alias to enter the container
## Setup the openvins container
* setup
    ```bash
    cd $HOME/d2dtracker_jetson
    ./setup_openvins.sh
    ```
* `source $HOME/.bashrc`

* run `openvins` alias to enter the container

## Setup the d2dtracker container
* setup
    ```bash
    cd $HOME/d2dtracker_jetson
    ./setup_d2dtracker.sh
    ```
* `source $HOME/.bashrc`

* run `d2dtracker` alias to enter the container

# Run System
## isaac_ros_visual_slam

```bash
source $HOME/.bashrc
vslam_container
```
* To use Realsense camera (e.g. D455), connect it and run
```bash
ros2 launch isaac_ros_visual_slam vslam_realsense.launch.py
```

## open_vins
```bash
source $HOME/.bashrc
openvins
```
* To test `open_vins` on ERUOC dataset, you need to download its ros bag, and then play it
* in another terminal inside the `openvins-container` run the openvins estimator
```bash
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
```

* **IMPROTANT**:  To use `open_vins` with live IMU/camera system, you will need to calibrate your cameras using Kalibr, calibrate your IMU using `allan_ros2` package (only works with ROS foxy, for now), calibrate both cameras and IMU using Kalibr. Then, use the calibration files to update the `config.yaml` file of the open_vins. See below video link for a tutorial.
* **Reference**: https://docs.openvins.com/gs-tutorial.html
* **Video** : https://youtu.be/rBT5O5TEOV4

## d2dtracker
```bash
source $HOME/.bashrc
d2dtracker
```
* Inside the `d2dtracker-container`, there is a `bash.sh` script inside `$HOME/shared_volume`. Edit which moduleyou wish to run (set its flag to `True`), save and close it. Source the `bash.sh` script. Then, run
```bash
ros2 launch d2dtracker_system run_system.launch.py
```