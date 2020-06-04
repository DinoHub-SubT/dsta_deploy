# UGV Catkin Workspace

Setting up the catkin workspace for the UGV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UGV catkin build workspace.

- There are different docker containers for the different catkin workspaces.

Follow the instructions below to setup the UGV catkin workspace.

## UGV Simulation (Planning-PC, NUC) Catkin Workspace

### 1. Access Docker Container (optional)

        # ssh into the remote Azure VM (if not already logged in).Change `azure.ugv1` to the correct VM name
        # -- if you are not using Azure, you may skip this step.
        ssh azure.ugv1

        # enter the docker shell container on your local laptop host or Azure VM host
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name ugv-sim-shell

        # its okay to ignore the following error if you have not yet built the workspace:
        # -> 'bash: /home/developer/deploy_ws/devel/...: No such file or directory'

### 2. Build Common

The common catkin workspace sets up default `cmake` options.

        # == Common Catkin Workspace ==

        # go to the `common` catkin workspace inside the docker container
        cd ~/deploy_ws/src/common

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set ugv

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

### 3. Build UGV Catkin Workspace

The UGV planning-pc catkin workspace contains all repositories that are running on the robot, on the planning-pc.
The UGV nuc catkin workspace contains all repositories that are running on the robot, on the nuc.

        # go to the `ugv` catkin workspace
        cd ~/deploy_ws/src/ugv/

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set sim

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

### 4. Build Simulation Catkin Workspace

The UGV simulation catkin workspace contains all repositories related in running the `ugv` in simulation.

The `ugv` catkin workspaces sets up default `cmake` options.

        # go to the simulation:darpa catkin workspace
        cd ~/deploy_ws/src/simulation/darpa/catkin

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set ugv

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

        # go to the simulation catkin workspace
        cd ~/deploy_ws/src/simulation

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set ugv

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

### 5. Build SubT Launch Catkin Workspace

The subt launch catkin workspace contains a centralized top-level launch.

        # go to the `subt_launch` catkin workspace
        cd ~/deploy_ws/src/subt_launch

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set ugv-sim

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

        # exit the container
        exit

## Cleanup (optional)

You should remove containers when done with its development.

        # stop the running container
        docker stop ppc-shell

        # remove the container
        docker rm ppc-shell

        # stop the running container
        docker stop nuc-shell

        # remove the container
        docker rm nuc-shell

        # stop the running container
        docker stop ugv-sim-shell

        # remove the container
        docker rm ugv-sim-shell

- The above steps will remove the containers.

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash [container-name]` command will enter a stopped container.

## Summary

You should now have a built `UGV` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
