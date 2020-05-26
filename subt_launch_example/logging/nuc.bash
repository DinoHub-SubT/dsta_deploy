#!/usr/bin/env bash

# create & go to the logging directory
date=$(date '+%Y-%m-%d')
mkdir -p $subt_rosbag_dir/$date/
cd $subt_rosbag_dir/$date/

# rosbag record specific topics
rosbag record --split -b 1024 --size 1024 __name:=rosbag_recording_nuc -o nuc \
    /aft_mapped_to_init \
    /velodyne_points \
    /diagnostics \
    /diagnostics_agg \
    /health/stats/nuc \
    /health/times/nuc \
    /hms_variable_publisher \
    /imu/data \
    /initialpose \
    /integrated_to_init \
    /integrated_to_map \
    /joy \
    /key_pose_path \
    /key_pose_to_map \
    /loop_closed_to_map \
    /navigation_boundary \
    /relative_pose_to_key_pose \
    /rosout_agg \
    /rostime/developer \
    /rostime/nuc \
    /rostime/robot \
    /rostime/xavier \
    /tf \
    /tf_static \
    /timediff/nuc_robot \
    /timediff/nuc_xav \
    /velodyne_cloud_key_pose \
    /velodyne_cloud_registered \
