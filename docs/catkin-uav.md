# UAV Catkin Workspace

Setting up the catkin workspace for the UAV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UAV catkin workspace.

- There are different docker containers for the different catkin workspaces.

Follow the instructions below to setup the UAV catkin workspace.

## UAV Simulation Workspace

### 1. Access Docker Container (optional)

**If you are not using docker containers, you may skip this step.**

**If you are using an Azure VM, remember to ssh into the VM first.**

        # enter the docker shell container on your local laptop host or Azure VM host
        #   -- its okay to ignore the error if you have not yet built the workspace: error is: 'bash: /home/developer/deploy_ws/devel/...: No such file or directory'
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

The UAV simulation catkin workspace contains all repositories related in running the `uav` in simulation.

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

## Cleanup (optional)

You should remove containers when done with its development.

        # stop the running container
        docker stop uav-sim-shell

        # remove the running container
        docker rm uav-sim-shell

- The above steps will remove the containers.

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash [container-name]` command will enter a stopped container.

## Summary

You should now have a built `UAV` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.