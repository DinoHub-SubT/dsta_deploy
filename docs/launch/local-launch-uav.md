# UAV Launch

## 1. Launch UAV Simulation

If you are using an Azure VM, you need to run these commands using remote-desktop or teamviewer.

        # enter the docker shell container (if not already joined)
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name uav-cpu-shell

        # load the tmux session. Example launch `uav1`
        ROBOT=uav1 tmuxp load ~/deploy_ws/src/subt_launch/tmux/localhost/uav.yaml

## 2. Verify Launch

Please verify all the launch scripts in the tmux sessions started.

- Node error might be okay, but if a launch failed to come up that is not okay.

## 3. UAV Simulation Commands

**Load GUI Config**

Select the left hand side "Open Config" that shows `gui_config.yaml`

Select the config file: `uav/core/rqt_behavior_tree_command/config/gui_config.yaml`

**UAV Explore**

Wait for both Gazebo & RViz to start.

Select the "Autonomously Explore" GUI button for the drone to explore.
