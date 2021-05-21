# UAV Launch

## 1. Launch UAV Simulation

```text
# enter the docker shell container (if not already joined)
docker-join.bash -n uav1-shell

# load the tmux session. Example launch `uav1`
ROBOT=uav1 tmuxp load ~/deploy_ws/src/subt_launch/tmux/simulation/uav.yaml
```

## 2. Verify Launch

Please verify all the launch scripts in the tmux sessions started.

- Node error might be okay, but if a launch failed to come up that is not okay.

- Feel free to close any extra windows for better performance if necessary. Essential windows are
    - Core rqt
    - Gazebo
    - Cave RViz 

## 3. UAV Simulation Commands

**Load GUI Config**

In Core rqt, pan the behaviour tree into full view.

Select the bottom left hand side "Open Config" that shows `gui_config.yaml`

Select the config file: `uav/core/rqt_behavior_tree_command/config/gui_config.yaml`

You will see a new set of GUI buttons appear.

**UAV Explore**

Wait for both Gazebo & Cave RViz to start.

Select the "Autonomously Explore" GUI button for the drone to explore.

On RViz, press "G" on the keyboard to give it a different waypoint for the drone to follow.

**Multi Robot Explore**

Please enable map sharing options: `src/uav/core/map_processor/config/base.yaml`

By setting the following configurations:

    use_local_origin_create_no_fly:     true
    manually_set_DARPA_tf:              true

## 4. Reset Simulation

In the terminal,
Press `CTRL-B` follow by `:`
Type `kill-server`

Back in the docker shell container,
Type `pkill -f ros`

Reload the tmux session
