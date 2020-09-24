# Basestation Launch

## 1. Access Docker Container

If you are using an Azure VM, remember to remote-desktop into the VM first.

If you are not using docker containers, you may skip this step.

        # enter the docker shell container on your local laptop host or Azure VM host
        docker-join.bash --name basestation-cpu-shell

## 2. Launch Basestation

        # load the tmux session
        ROBOT=basestation tmuxp load ~/deploy_ws/src/subt_launch/tmux/localhost/basestation.yaml

## Summary

Please launch the other VMs (ugv, uav), to see the robots on the basestation RViz.
