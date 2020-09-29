# Basestation Launch

## 1. Access Docker Container

        # enter the docker shell container (if not already joined)
        docker-join.bash --name basestation-cpu-shell

        # Load the tmux session. Example launch `ugv1`
        ROBOT=basestation tmuxp load ~/deploy_ws/src/subt_launch/tmux/localhost/basestation.yaml

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

## 3. Launch Basestation

        # load the tmux session
        ROBOT=basestation tmuxp load ~/deploy_ws/src/subt_launch/tmux/localhost/basestation.yaml

## Summary

Please launch the other VMs (ugv, uav), to see the robots on the basestation RViz.
