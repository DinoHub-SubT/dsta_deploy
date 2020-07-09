# UGV Launch

## UGV Catkin Workspace

### 1. Azure Access

        # ssh into the remote VM. Example:
        ssh azure.ugv1

        # view teamviewer info
        sudo teamviewer info

        # get the teamviewer ID
        sudo teamviewer info | grep "TeamViewer ID"

Copy the remote `TeamViewer ID` into your localhost teamviewer "Control Remote Computer" Partner ID.

- *teamviewer password:* `teamviewer`

Once in the remote TeamViewer Window, access the `subt` user's desktop

- Azure VM user `subt` password is: `Password1234!`

If you do not prefer to use Teamveiwer, you can use RDP instead.

### 4. Access Docker Container

        # ssh into the remote Azure VM (if not already logged in)
        # -- if you are not using Azure, you may skip this step.
        ssh azure.ugv1

        # enter the docker shell container (if not already joined)
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name ugv-sim-shell

### 4. Edit Launch Files

Edit Launch File: `~/deploy_ws/src/simulation/darpa/catkin/darpa_subt/x1_control/launch/control.launch`

        # Change
        #       <rosparam command="load" file="$(arg config_extras)" />
        # To:
        #       <!-- <rosparam command="load" file="$(arg config_extras)" /> -->


Edit Launch File: `~/deploy_ws/src/ugv/nuc/local_planner/launch/local_planner.launch`

        # Change
        #       <remap from="/X1/cmd_vel" to="/auton_twist"/>
        # To:
        #       <!--remap from="/X1/cmd_vel" to="/auton_twist"/-->

### 5. Launch UGV Simulation

        # ssh into the remote Azure VM (if not already logged in)
        # -- if you are not using Azure, you may skip this step.
        ssh azure.ugv1

        # enter the docker shell container on your local laptop host or Azure VM host
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name ugv-sim-shell

        # go to the top-level deploy workspace
        cd ~/deploy_ws/src

        # Load the tmux session. Example launch `ugv1`
        ROBOT=ugv1 tmuxp load subt_launch/tmux/azure/ugv.yaml

        # (OPTIONAL) open a new tab/window and publish a waypoint
        rostopic pub --once /way_point geometry_msgs/PointStamped '{header: {frame_id: map}, point: {x: 16, y: 0, z: 0}}'

        # (OPTIONAL) Move the UGV using the Basestation Azure VM GUI.
        # -- On the Basestation Azure VM (example, moving ugv1):
        #       Select UGV1 on both control GUIs
        #       Select waypoints on rviz

### 6. Verify Launch

Please verify all the launch scripts in the tmux sessions do not have any errors.

If you see any launch that failed to come up, please refer to the ugv's `operations/launch/tmuxp/sim/ugv[1-N].yaml` to see which failed and then relaunch manually.

### 7. Transfer To Changes (optional)

The changes, outlined in this tutorial can all be done on the localhost (so you dont need to do these changes on every robot manually).

Once changed on the localhost, you can then `transfer.to` to transfer the changes from your localhost to the remote:

        # ugv transfer.to command
        ./deployer -r azure.ugv1.transfer.to

If you find the `transfer.to` is too slow or not updating files during a transfer, you can change the the `transfer.to` options in the file:

        operations/deploy/scenarios/.ugv.env

You can edit the option: `deploy_rsync_opts`

- This option tells the deployer to **exclude** files during the transfer. You may change the files that get excluded.
- **Example change:** adding `--exclude=src/.git`, will reduce the time for the transfer, but you wont see any git changes reflected on the remote.

### Summary

You should now be able to control the robot movement using the buttons in the basestation.
