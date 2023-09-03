#!/bin/bash
alias d2d='ros2 launch d2dtracker_system run_system.launch.py'
alias cbs='colcon  build --packages-select'
alias cb='colcon build'

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=17

export RUN_XRCE=False
export RUN_REALSENSE=False
export RUN_SLAM=False
export RUN_PX4_ROS=False
export RUN_KF=False
export RUN_TRAJ_PRED=False
export RUN_YOLO=False
export RUN_YOLO_POSE=False


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
line_to_check="alias viz='rviz2 -d /home/d2d/shared_volume/ros2_ws/src/open_vins/ov_msckf/launch/display_ros2.rviz'"

if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

source $HOME/.bashrc