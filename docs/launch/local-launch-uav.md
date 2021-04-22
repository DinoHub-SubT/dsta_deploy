# UAV Launch

## 1. Configure Simulation Parameters

To enable map_processor coordination visualization rviz parameters, please up the following rosparams:

        gedit ~/dsta_deploy/src/uav/core/map_processor/config/base.yaml

        # ----------------------------
        # change
        manually_set_DARPA_tf: false
        # to
        manually_set_DARPA_tf: true

        # ----------------------------
        # change
        use_local_origin_create_no_fly: false
        # to
        use_local_origin_create_no_fly: true


## 2. Launch UAV Simulation

        # enter the docker shell container (if not already joined)
        docker-join.bash -n uav1-shell

        # load the tmux session. Example launch `uav1`
        ROBOT=uav1 tmuxp load ~/deploy_ws/src/subt_launch/tmux/simulation/uav.yaml

## 3. Verify Launch

Please verify all the launch scripts in the tmux sessions started.

- Node error might be okay, but if a launch failed to come up that is not okay.

## 4. UAV Simulation Commands

**Load GUI Config**

Select the left hand side "Open Config" that shows `gui_config.yaml`

Select the config file: `uav/core/rqt_behavior_tree_command/config/gui_config.yaml`

**UAV Explore**

Wait for both Gazebo & RViz to start.

Select the "Autonomously Explore" GUI button for the drone to explore.

On RViz, press "G" on the keyboard to give it a different waypoint for the drone to follow.
