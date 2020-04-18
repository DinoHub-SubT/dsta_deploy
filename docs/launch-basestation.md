# Basestation Launch

## Basestation Catkin Workspace

### 1. Access Docker Container

If you are using an Azure VM, remember to ssh into the VM first.

If you are not using docker containers, you may skip this step.

        # enter the docker shell container on your local laptop host or Azure VM host
        docker-join.bash --name gui-shell

### 2. Launch Basestation

        # load the tmux session
        tmuxp load operations/launch/tmuxp/sim/basestation.yaml
