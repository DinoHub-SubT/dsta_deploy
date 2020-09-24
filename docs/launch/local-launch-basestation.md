# Basestation Launch

## 1. Access Docker Container

        # enter the docker shell container (if not already joined)
        docker-join.bash --name basestation-cpu-shell

## 2. Launch Basestation

        # load the tmux session
        ROBOT=basestation tmuxp load ~/deploy_ws/src/subt_launch/tmux/localhost/basestation.yaml

## Summary

Please launch the other VMs (ugv, uav), to see the robots on the basestation RViz.
