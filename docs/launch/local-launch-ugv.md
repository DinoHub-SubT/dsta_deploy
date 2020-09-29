# UGV Launch

## 1. Launch UGV Simulation

        # enter the docker shell container on your local laptop host
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name ugv-sim-shell

        # Load the tmux session. Example launch `ugv1`
        ROBOT=ugv1 tmuxp load ~/deploy_ws/src/subt_launch/tmux/localhost/ugv.yaml

        # (OPTIONAL) open a new tab/window and publish a waypoint
        rostopic pub --once /way_point geometry_msgs/PointStamped '{header: {frame_id: map}, point: {x: 16, y: 0, z: 0}}'

        # (OPTIONAL) Move the UGV using the Basestation GUI.
        # - On the Basestation GUI:
        #       Select UGV1 on both control GUIs
        #       Select waypoints on rviz

## 2. Control UGV Using Basestation GUI (optional)

You will need to change some ROS params in both the ugv & basestation containers:

Step 1:

        # unset the ROS params in the UGV docker container
        docker-join.bash --name ugv-sim-shell
        unset ROS_MASTER_IP
        unset ROS_MASTER_URI
        unset ROS_HOSTNAME

        # exit the container
        exit

Step 2:

        # assuming you have already built the basestation GUI
        # unset the ROS params in the UGV docker container
        docker-join.bash --name basestation-cpu-shell
        unset ROS_MASTER_IP
        unset ROS_MASTER_URI
        unset ROS_HOSTNAME

        # exit the container
        exit

Now you can launch the UGV and Basestation.

- If you *delete* the docker containers you must reset these ROS param options.
- If you stop the container, you do not need to reset these ROS param options.

This will be automated in the next iteration of docker cleanup. For now, you must manually do these steps.

## 3. Verify Launch

Please verify all the launch scripts in the tmux sessions do not have any errors.

## Summary

You should now be able to control the robot movement using the buttons in the basestation GUI.

    - Please build the localhost basestation setup to control via basestation GUI.
