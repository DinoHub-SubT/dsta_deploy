#!/usr/bin/env bash

# create & go to the logging directory
date=$(date '+%Y-%m-%d')
mkdir -p $subt_rosbag_dir/$date/
cd $subt_rosbag_dir/$date/

# rosbag record specific topics
rosbag record --split -b 1024 --size 1024 __name:=rosbag_recording_planning_pc -o planning-pc \
    /motor_status \
    /twist \
    /auton_twist \
    /joy_twist \
    /joy \
    /speed \
    /rosout_agg \
    /twist_velocity_cps \
    /stuck_status \
    /recover_stuck \
    /stuck_recovery_status \
    /stuck_recovery_maneuver_twist \
    /ugv_imu/data \
    /ugv_imu/rpy \
    /tip_ratios \
    /twist_slow_factor \
    /recover_tip_over \
    /tip_over_recovery_status \
    /tip_over_recovery_maneuver_twist \
    /twist_mux_topic \
    /behavior_tree_graphviz \
    /check_obstacle \
    /cloud_clearing \
    /comms_planner_confirm \
    /comms_planner_drop \
    /comms_status \
    /coordination_polygon \
    /dfs_planner_markers \
    /dfs_planner_mode \
    /path \
    /pause \
    /planner_mode \
    /radio_command \
    /return_home \
    /speed \
    /status_update \
    /stuck_recovery_maneuver_status \
    /tip_over_recovery_maneuver_status \
    /tip_ratios \
    /traj \
    /ugv1/two_way_drive \
    /ugv1/two_way_drive_mode \
    /ugv2/two_way_drive \
    /ugv2/two_way_drive_mode \
    /ugv_imu/data \
    /ugv_imu/rpy \
    /way_point \
    /tf \
    /tf_static \
    /integrated_to_map \
    /velodyne_cloud_terrain \
    /graph_planner_command \
    /grpah_planner_in_progress \
    /ignore_polygon_boundary \
    /free_paths