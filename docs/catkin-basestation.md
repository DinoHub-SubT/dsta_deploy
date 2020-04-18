# Basestation Catkin Workspace

Setting up the catkin workspace for the Basestation workspaces requires using the `catkin` tool.

There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UAV catkin build workspace.

Assuming you have already setup all your basestation docker containers, please follow the instructions below to setup the UAV catkin workspace.

* * *

## Basestation Catkin Workspace

### 1. Access Docker Container

If you are using an Azure VM, remember to ssh into the VM first.

If you are not using docker containers, you may skip this step.

        # enter the docker shell container on your local laptop host or Azure VM host
        docker-join.bash --name gui-shell

### 2. Build UAV Common

The common catkin workspace sets up default `cmake` options.

        # == Common Catkin Workspace ==

        # go to the `common` catkin workspace inside the docker container
        cd ~/deploy_ws/src/common

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set system76-pc

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

### 3. Build Basestation Catkin Workspace

The basestation catkin workspace contains all repositories that are running during SubT, on the basestation.

        # go to the `basestation` catkin workspace
        cd ~/deploy_ws/src/basestation

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set system76-pc

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

        # exit the container
        exit

        # stop the running container
        docker stop gui-shell

* * *

## Cleanup

You should remove containers when done with its development.

        # stop the running container
        docker stop gui-shell

        # remove the container
        docker rm gui-shell

## Summary

You should now have a built `Basestation` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.

Please go back to [`build-tutorial`](build-tutorial.md#Summary) for summary comments.
