# UAV Catkin Workspace

Setting up the catkin workspace for the UAV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UAV catkin workspace.

- There are different docker containers for the different catkin workspaces.

Follow the instructions below to setup the UAV catkin workspace.

## Local CPU VM

### 1. Catkin Build

Follow this step, **on the localhost**, not on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the docker shell on the remote host
        ./deployer -s local.uav.cpu.docker.shell

        # clean the previous built workspaces
        ./deployer -s local.uav.cpu.catkin.clean

        # build the PX4 firmware
        ./deployer -s local.uav.cpu.px4

        # catkin build the UGV workspaces
        ./deployer -s local.uav.cpu.catkin.build

- Please change the robot name `uav` to whichever Azure robot VM you are building on.

## Local GPU VM

### 1. Catkin Build

Follow this step, **on the localhost**, not on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the docker shell on the remote host
        ./deployer -s local.uav.gpu.docker.shell

        # clean the previous built workspaces
        ./deployer -s local.uav.gpu.catkin.clean

        # build the PX4 firmware
        ./deployer -s local.uav.gpu.px4

        # catkin build the UGV workspaces
        ./deployer -s local.uav.gpu.catkin.build

- Please change the robot name `uav` to whichever Azure robot VM you are building on.

## Cleanup (optional)

You should remove containers when done with its development (for those that are available).

Automated remove the docker containers:

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # stop the docker container
        ./deployer -s local.uav.docker.stop

        # remove the docker container
        ./deployer -s local.uav.docker.remove

Or manually remove the docker containers:

        # stop the running container
        docker stop uav-cpu-shell uav-gpu-shell

        # remove the running container
        docker rm -f uav-cpu-shell uav-gpu-shell

- The above steps will remove the containers.

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash [container-name]` command will enter a stopped container.

## Summary

You should now have a built `UAV` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
