# UAV Catkin Workspace

Setting up the catkin workspace for the UAV workspaces requires using the `catkin` tool.

There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UAV catkin build workspace.

There are different docker containers for the different catkin workspaces.

Assuming you have already setup your docker container, please follow the instructions below to setup the UAV catkin workspace.

## UAV Simulation Catkin Workspace

### 1. Access Docker Container

If you are using an Azure VM, remember to ssh into the VM first.

If you are not using docker containers, you may skip this step.

        # enter the docker shell container on your local laptop host or Azure VM host
        docker-join.bash --name uav-sim-shell

### 2. Build Common

The common catkin workspace sets up default `cmake` options.

        # == Common Catkin Workspace ==

        # go to the `common` catkin workspace inside the docker container
        cd ~/deploy_ws/src/common

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set uav

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

### 3. Build UAV Simulation Catkin Workspace

The UAV simulation catkin workspace contains all repositories related in running the uav in simulation.

The `uav:sim` catkin workspace sets up default `cmake` options.


        # go to the uav:sim catkin workspace
        cd ~/deploy_ws/src/uav/sim

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set sim

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

        # exit the container
        exit

* * *

## Cleanup

You should remove containers when done with its development.

        # stop the running container
        docker stop uav-sim-shell

        # remove the running container
        docker rm uav-sim-shell

## Summary

You should now have a built `UAV` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.

Please go back to [`build-tutorial`](build-tutorial.md#Summary) for summary comments.
