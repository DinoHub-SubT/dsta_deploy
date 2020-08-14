# UAV Launch

## 1. Launch UAV Simulation

If you are using an Azure VM, you need to run these commands using remote-desktop or teamviewer.

        # enter the docker shell container (if not already joined)
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name uav-sim-shell

        # load the tmux session. Example launch `uav1`
        ROBOT=uav1 tmuxp load ~/deploy_ws/src/subt_launch/tmux/localhost/uav.yaml

## 2. Verify Launch

Please verify all the launch scripts in the tmux sessions started.

- Node error might be okay, but if a launch faile to come up that is not okay.

- If you see any launch that failed to come up, please refer to the uav's `subt_launch/launch/tmuxp/sim/uav[1-N].yaml` to see which launch failed and then relaunch manually.

## 3. UAV Simulation Commands

**Load GUI Config**

Select the left hand side "Open Config" that shows `gui_config.yaml`

Select the config file: `uav/core/rqt_behavior_tree_command/config/gui_config.yaml`

**Load Trajectory Config**

Select the right hand side "Open Config" that shows `fixed_trajectories.yaml`

Select the config file: `uav/core/core_trajectory_library/config/fixed_trajectories.yaml`

**UAV Takeoff**

Select the "Takeoff" GUI button for the drone to take off.

- you should see the drone moving in RVIZ.

**UAV Explore**

Select the "Explore" or "Autonomously Explore" GUI button for the drone to explore.

Or go to the Basestation VM and give the drone a waypoint.

## Summary

You should now be able to control the robot movement using the buttons in the basestation.

- Select "waypoint" in rviz on the basestation VM.

**Reference:** the repos, install setup, launch instructions can be found in the [UAV readme](https://bitbucket.org/castacks/core_central/src/subt_nuc_velodyne_small_drone/).