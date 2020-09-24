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

## 2. Verify Launch

Please verify all the launch scripts in the tmux sessions do not have any errors.

## Summary

You should now be able to control the robot movement using the buttons in the basestation GUI.

    - Please build the localhost basestation setup to control via basestation GUI.
