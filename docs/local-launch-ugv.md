# UGV Launch

## 1. Access Docker Container

        # enter the docker shell container (if not already joined)
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name ugv-sim-shell

## 2. Edit Launch Files

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

## 3. Launch UGV Simulation

        # enter the docker shell container on your local laptop host or Azure VM host
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name ugv-sim-shell

        # Load the tmux session. Example launch `ugv1`
        ROBOT=ugv1 tmuxp load ~/deploy_ws/src/subt_launch/tmux/localhost/ugv.yaml

        # (OPTIONAL) open a new tab/window and publish a waypoint
        rostopic pub --once /way_point geometry_msgs/PointStamped '{header: {frame_id: map}, point: {x: 16, y: 0, z: 0}}'

        # (OPTIONAL) Move the UGV using the Basestation Azure VM GUI.
        # -- On the Basestation Azure VM (example, moving ugv1):
        #       Select UGV1 on both control GUIs
        #       Select waypoints on rviz

## 4. Verify Launch

Please verify all the launch scripts in the tmux sessions do not have any errors.

If you see any launch that failed to come up, please refer to the ugv's `subt_launch/launch/tmuxp/sim/ugv[1-N].yaml` to see which failed and then relaunch manually.

## Summary

You should now be able to control the robot movement using the buttons in the basestation.
