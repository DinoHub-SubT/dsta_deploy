# UAV Launch

## 1. Azure Access

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

## 2. Launch UAV Simulation

If you are using an Azure VM, you need to run these commands using remote-desktop or teamviewer.

        # ssh into the remote Azure VM (if not already logged in)
        # -- if you are not using Azure, you may skip this step.
        ssh azure.uav1

        # enter the docker shell container (if not already joined)
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name uav-sim-shell

        # load the tmux session. Example launch `uav1`
        ROBOT=uav1 tmuxp load ~/deploy_ws/src/subt_launch/launch/tmuxp/azure/uav1.yaml

## 3. Verify Launch

Please verify all the launch scripts in the tmux sessions started.

- Node error might be okay, but if a launch faile to come up that is not okay.

- If you see any launch that failed to come up, please refer to the uav's `subt_launch/launch/tmuxp/sim/uav[1-N].yaml` to see which launch failed and then relaunch manually.

## 4. UAV Simulation Commands

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

## 5. Transfer To Changes (optional)

The changes, outlined in this tutorial can all be done on the localhost (so you dont need to do these changes on every robot manually).

Once changed on the localhost, you can then `transfer.to` to transfer the changes from your localhost to the remote:

        # uav transfer.to command
        ./deployer -r azure.uav1.transfer.to

If you find the `transfer.to` is too slow or not updating files during a transfer, you can change the the `transfer.to` options in the file:

        operations/deploy/scenarios/.uav.env

You can edit the option: `deploy_rsync_opts`

- This option tells the deployer to **exclude** files during the transfer. You may change the files that get excluded.
- **Example change:** adding `--exclude=src/.git`, will reduce the time for the transfer, but you wont see any git changes reflected on the remote.

## Summary

You should now be able to control the robot movement using the buttons in the basestation.

- Select "waypoint" in rviz on the basestation VM.

**Reference:** the repos, install setup, launch instructions can be found in the [UAV readme](https://bitbucket.org/castacks/core_central/src/subt_nuc_velodyne_small_drone/).