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
* **Setup nvidia visual slam software** Enter the repo's directory and execute the `setup_vslam.sh` script
    ```bash
    cd $HOME/d2dtracker_jetson
    ./setup_vslam.sh
    ```
    **NOTE: Provide `sudo` password when asked.**

    `source $HOME/.bashrc`

    run `vslam_container` alias to enter the container
* **Setup the openvins container**
    ```bash
    cd $HOME/d2dtracker_jetson
    ./setup_openvins.sh
    ```
    `source $HOME/.bashrc`

    run `openvins` alias to enter the container

* **Setup the d2dtracker container**
    ```bash
    cd $HOME/d2dtracker_jetson
    ./setup_d2dtracker.sh
    ```
    `source $HOME/.bashrc`

    run `d2dtracker` alias to enter the container

# Run System
## isaac_ros_visual_slam

```bash
source $HOME/.bashrc
vslam_container
```

## open_vins
```bash
source $HOME/.bashrc
openvins
```

## d2dtracker
```bash
source $HOME/.bashrc
d2dtracker
```