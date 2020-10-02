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

Assuming ROS MASTER will be on the UGV docker container.

Step 1:

        # unset the ROS params in the UGV docker container
        docker-join.bash --name ugv-sim-shell

        # set the hostname, ignore error message 'sudo: unable to resolve host ...'
        sudo hostname ugv

        # set the ROS MASTER URI to the ugv
        export ROS_MASTER_URI=http://ugv:11311

        # exit the container (to apply the changed)
        exit

Step 2:

        # assuming you have already built the basestation GUI
        # unset the ROS params in the UGV docker container
        docker-join.bash --name basestation-cpu-shell

        # set the hostname, ignore error message 'sudo: unable to resolve host ...'
        sudo hostname basestation

        # set the ROS MASTER URI to the ugv
        export ROS_MASTER_URI=http://ugv:11311

        # exit the container (to apply the changed)
        exit

Enter the containers again and launch the UGV and Basestation.

Comments:

- Please **run the UGV launch first**, before the basestation.

- If you sometimes do not see the tree, then stop the container and reset the above configs.

- When you stop the container, you must reset the everything again.

Unfortunately for now, you must manually do these steps. This **will be fixed & automated** in the next iteration of docker cleanup.

## 3. Verify Launch

Please verify all the launch scripts in the tmux sessions do not have any errors.

## Summary

You should now be able to control the robot movement using the buttons in the basestation GUI.

- Please build the localhost basestation setup to control via basestation GUI.
