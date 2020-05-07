# UAV Launch

## UAV Catkin Workspace

### 1. Azure Access

        # ssh into the remote VM. Example:
        ssh azure.uav1

        # view teamviewer info
        sudo teamviewer info

        # get the teamviewer ID
        sudo teamviewer info | grep "TeamViewer ID"

Copy the remote `TeamViewer ID` into your localhost teamviewer "Control Remote Computer" Partner ID.

- *teamviewer password:* `teamviewer`

Once in the remote TeamViewer Window, access the `subt` user's desktop

- Azure VM user `subt` password is: `Password1234!`

If you do not prefer to use Teamveiwer, you can use RDP instead.

### 2. Verify Communication Manager Connection

Please verify you have setup the azure vm communication manager connection IPs

        # ssh into the remote VM (if not already logged in)
        ssh azure.uav1

        # open the communication manager config
        gedit ~/deploy_ws/src/common/communication_manager/config/USER_QOS_PROFILES.xml

        # make sure the IPs are added to every 'initial_peers' tag:
        
        <element>10.3.1.1</element>
        <element>10.3.1.11</element>
        <element>10.3.1.12</element>
        <element>10.3.1.13</element>
        <element>10.3.1.51</element>
        <element>10.3.1.52</element>
        <element>10.3.1.53</element>
        <element>10.3.1.54</element>

### 3. Launch UAV Simulation

If you are using an Azure VM, you need to run these commands using remote-desktop or teamviewer.

        # ssh into the remote Azure VM (if not already logged in)
        # -- if you are not using Azure, you may skip this step.
        ssh azure.uav1

        # enter the docker shell container (if not already joined)
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name uav-sim-shell

        # go to the deploy repo
        cd ~/deploy_ws/src/

        # load the tmux session. Example launch `uav1`
        tmuxp load operations/launch/tmuxp/sim/uav1.yaml

### 4. Verify Launch

Please verify all the launch scripts in the tmux sessions started.

- Node error might be okay, but if a launch faile to come up that is not okay.

- If you see any launch that failed to come up, please refer to the uav's `operations/launch/tmuxp/sim/uav[1-N].yaml` to see which launch failed and then relaunch manually.

### 5. UAV Simulation Commands

**Load GUI Config**

Select the left hand side "Open Config" that shows `gui_config.yaml`

Select the config file: `uav/sim/rqt_behavior_tree_command/config/gui_config.yaml`

**Load Trajectory Config**

Select the right hand side "Open Config" that shows `fixed_trajectories.yaml`

Select the config file: `uav/sim/core_trajectory_library/config/fixed_trajectories.yaml`

**UAV Takeoff**

Select the "Takeoff" GUI button for the drone to take off.

- you should see the drone moving in RVIZ.

**UAV Explore**

Select the "Explore" or "Autonomously Explore" GUI button for the drone to explore.

Or go to the Basestation VM and give the drone a waypoint.

### Summary

You should now be able to control the robot movement using the buttons in the basestation.

- Select "waypoint" in rviz on the basestation VM.

**Reference:** the repos, install setup, launch instructions can be found in the [UAV readme](https://bitbucket.org/castacks/core_central/src/subt_nuc_velodyne_small_drone/).