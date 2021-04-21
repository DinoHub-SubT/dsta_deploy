# UAV Launch

## 1. Azure Access

### Teamviewer

        # ssh into the remote VM.
        ssh azure.uav1

        # view teamviewer info
        sudo teamviewer info

        # get the teamviewer ID
        sudo teamviewer info | grep "TeamViewer ID"

Copy the remote `TeamViewer ID` into your localhost teamviewer "Control Remote Computer" Partner ID.

- *teamviewer password:* `teamviewer`

Once in the remote TeamViewer Window, access the `subt` user's desktop

- Azure VM user `subt` password is: `Password1234!`

### RDP

If you do not prefer to use Teamveiwer, you can use RDP instead.

        subt tools rdp -t azure-uav1-window -h azure-uav1 -u subt -p Password1234!

## 2. Access Docker Container

        # ssh into the remote Azure VM
        ssh azure.uav1

        # enter the docker shell container
        docker-join.bash -n uav1-shell

## 3. Launch UAV Simulation

        # load the tmux session. Example launch `uav1`
        ROBOT=uav1 tmuxp load ~/deploy_ws/src/subt_launch/tmux/simulation/uav.yaml

## 4. Verify Launch

Please verify all the launch scripts in the tmux sessions started.

- Node error might be okay, but if a launch failed to come up that is not okay.

## 3. UAV Simulation Commands

**Load GUI Config**

Select the left hand side "Open Config" that shows `gui_config.yaml`

Select the config file: `uav/core/rqt_behavior_tree_command/config/gui_config.yaml`

**UAV Explore**

Wait for both Gazebo & RViz to start.

Select the "Autonomously Explore" GUI button for the drone to explore.

On RViz, press "G" on the keyboard to give it a different waypoint for the drone to follow.

## Summary

You should now be able to control the robot movement using the buttons in the basestation.

- Select "waypoint" in rviz on the basestation VM.

**Reference:** the repos, install setup, launch instructions can be found in the [UAV readme](https://bitbucket.org/castacks/core_central/src/subt_nuc_velodyne_small_drone/).
