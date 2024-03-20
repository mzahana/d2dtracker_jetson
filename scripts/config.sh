#!/bin/bash

if [ -f "/opt/ros/humble/install/setup.bash" ]; then
    echo "sourcing /opt/ros/humble/install/setup.bash"
    source /opt/ros/humble/install/setup.bash
fi
if [ -f "$HOME/shared_volume/ros2_ws/install/setup.bash" ]; then
    echo "sourcing $HOME/ros2_ws/install/setup.bash"
    source $HOME/shared_volume/ros2_ws/install/setup.bash
fi

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=17

#
# Modules
#
export RUN_XRCE=False
export RUN_REALSENSE=False
export RUN_SLAM=False
export RUN_PX4_ROS=False
export RUN_KF=False
export RUN_TRAJ_PRED=False
export RUN_YOLO=False
export RUN_YOLO_POSE=False
export RUN_OPENVINS=False
export RUN_MAVROS=False
export RUN_MAVROS_TFS=False
export RUN_ARDUCAM_STEREO=False
export RUN_APRILTAG=False
export RUN_GRU_PATH_PREDICTOR=False
export RUN_MPC_TRAJ_GENERATION=False
export RUN_GEOMETRIC_CONTROLLER=False

#
# MAVROS variables
#
export MAVROS_FCU_URL='/dev/ttyUSB0:921600'
# Example: export MAVROS_GCS_URL='udp://:14550@192.168.1.151:14550'
export MAVROS_GCS_URL=''
# Example: export MAVROS_PLUGINLIST_YAML='/home/d2d/shared_volume/ros2_ws/src/d2dtracker_system/config/px4_pluginlists.yaml'
export MAVROS_PLUGINLIST_YAML=''
# export MAVROS_PX4_YAML='/home/d2d/shared_volume/ros2_ws/src/d2dtracker_system/config/px4_config.yaml'
export MAVROS_PX4_YAML=''
export MAVLINK_ID='1'
#
# OpenVins
#
# Example: export OPENVINS_YAML='/home/d2d/shared_volume/ros2_ws/src/open_vins/config/custom_d455/estimator_config.yaml'
export OPENVINS_YAML=''

#
# Kalman filter
#
# Example: export KF_YAML='/home/d2d/shared_volume/ros2_ws/src/d2dtracker_system/config/kf_param.yaml'
export KF_YAML=''

#
# Yolov8
#
# Example export YOLOV8_MODEL_PATH='/home/d2d/shared_volume/ros2_ws/src/d2dtracker_drone_detector/config/drone_detection_v3.pt'
export YOLOV8_MODEL_PATH=''

#
# apriltag_ros variables
#
# Example export APRILTAG_PARAMS_YAML='/home/d2d/shared_volume/ros2_ws/src/d2dtracker_system/config/apriltag_params.yaml'
# Example export APRILTAG_TAGS_YAML='/home/d2d/shared_volume/ros2_ws/src/d2dtracker_system/config/tags.yaml'
export APRILTAG_PARAMS_YAML=''
export APRILTAG_TAGS_YAML=''
export APRILTAG_PARENT_FRAME='odom'

#
# drone_path_predictor_ros
#
# Example: GRU_YAML_FILE=/home/d2d/shared_volume/ros2_ws/src/d2dtracker_system/config/gru.yaml
export GRU_YAML_FILE=''

#
# trajectory_generation
#
# Example: MPC_TRAJ_GEN_YAML_FILE=/home/d2d/shared_volume/ros2_ws/src/d2dtracker_system/config/mpc_traj_gen.yaml
export MPC_TRAJ_GEN_YAML_FILE=''

#
# mav_controllers_ros (geometric controller)
#
# Example: GEOMETRIC_CONTROLLER_YAML_FILE=/home/d2d/shared_volume/ros2_ws/src/d2dtracker_system/config/geometric_controller.yaml
# Example: GEOMETRIC_MAVROS_YAML_FILE=/home/d2d/shared_volume/ros2_ws/src/d2dtracker_system/config/geometric_mavros.yaml
export GEOMETRIC_CONTROLLER_YAML_FILE=''
export GEOMETRIC_MAVROS_YAML_FILE=''

# The following code creates convenience aliases to be used inside the d2dtracker container
bashrc_file="$HOME/.bashrc"
line_to_check="alias viz='rviz2 -d /home/d2d/shared_volume/ros2_ws/src/open_vins/ov_msckf/launch/display_ros2.rviz'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias rs='ros2 launch realsense2_camera rs.launch.py'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias vins='ros2 launch ov_msckf subscribe.launch.py config_path:=/home/d2d/shared_volume/ros2_ws/src/open_vins/config/d455_custom/estimator_config.yaml'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias cb='colcon build'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias cbs='colcon  build --packages-select'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias d2d='ros2 launch d2dtracker_system run_system.launch.py'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

source $HOME/.bashrc