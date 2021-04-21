# UGV Launch

## 1. Launch UGV Simulation

        # enter the docker shell container on your local laptop host
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name ugv1-shell

        # Load the tmux session. Example launch `ugv1`
        ROBOT=ugv1 tmuxp load ~/deploy_ws/src/subt_launch/tmux/localhost/ugv.yaml

        # (OPTIONAL) open a new tab/window and publish a waypoint
        rostopic pub --once /way_point geometry_msgs/PointStamped '{header: {frame_id: map}, point: {x: 16, y: 0, z: 0}}'

        # (OPTIONAL) Move the UGV using the Basestation GUI.
        # - On the Basestation GUI:
        #       Select UGV1 on both control GUIs
        #       Select waypoints on rviz

## 2. Control UGV Using Basestation GUI (optional)

Please setup the localhost basestation, to control the ugvs using the basestation GUI.

- [`docker`](../docker/local-docker-basestation.md) basestation localhost tutorial
- [`catkin`](../catkin/local-catkin-basestation.md) basestation localhost tutorial
- [`launch`](local-launch-basestation.md) basestation localhost tutorial

Comments:

- If you do not see the tree, then restart the container. Otherwise, notify the maintainer.

## 3. Verify Launch

Please verify all the launch scripts in the tmux sessions started.

- Node error might be okay, but if a launch failed to come up that is not okay.

## 4. UGV Simulation Commands

Select the ugv name on the control panel.

Select the waypoint button on the RVIZ panel, then place a waypoint in RVIZ.

- You should now be able to control the robot movement using the buttons in the basestation GUI.

