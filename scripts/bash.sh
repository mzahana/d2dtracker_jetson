#!/bin/bash
alias d2d='ros2 launch d2dtracker_system run_system.launch.py'
alias cbs='colcon  build --packages-select'
alias cb='colcon build'

export RUN_XRCE=False
export RUN_REALSENSE=False
export RUN_SLAM=False
export RUN_PX4_ROS=False
export RUN_KF=False
export RUN_TRAJ_PRED=False
export RUN_YOLO=False
export RUN_YOLO_POSE=False
