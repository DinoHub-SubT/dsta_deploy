# Build UAV With Docker Walkthrough

The `SubT` workspace will be built and will run in docker containers.
    
- Workspace dependencies are installed in docker images.

**Note:** This tutorial will build the docker setup directly on the localhost.

* * *

## Quick Start

The quick start will walkthrough the following:

- How to install workspace dependencies in docker images on your localhost.
- How to create docker containers on your localhost.
- How to build the catkin workspaces in the docker containers.

### 0. Check the Scenario Files

Please check the scenario files: `operations/deploy/scenarios`

This build uses the `desktop-[project].env` files (even if building on azure).

### 1. Building Docker Images

Building docker images on the localhost will install all the workspace dependencies.

**UAV Docker Image**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # build the ugv planning-pc docker image
        ./deployer -s desktop.uav.docker.image

**Cleanup (Required)**

        # Remove any previously created docker containers (optional)
        #   - its okay to ignore error 'Error: No such container'  and continue to the next step.
        docker rm -f uav-sim-shell

        # cleanup dangling docker images
        docker rmi -f $(docker images -f "dangling=true" -q)

**Verify Docker Images** 

        # View the docker images built on the localhost
        docker images

        # verify you see the following docker images (in any order):
        #     ->
        #        subt/uav:sim
        #        subt/uav:ros

## 2. Creating Docker Containers With Shell Access

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything you would do normally do direclty on the host, but instead inside the docker container.

**UAV Docker Container Shell Access**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the ugv:planning-pc docker container
        ./deployer -s desktop.uav.docker.shell

**Verify Docker Containers** 

        # view running docker containers
        docker ps

        # verify you see the following docker containers (in any order):
        #   -> uav-shell

**Common Issues**

- Error `Unknown runtime specified nvidia`, please change:

        # open the scenario file
        gedit operations/deploy/scenarios/desktop-uav.env

        # change the variable 'GPU_ENABLE_TYPE' to be the below:
        export GPU_ENABLE_TYPE="non-gpu"

## 3. Building The Catkin Workspace

**Build UAV Simulation**

The uav simulation catkin workspace contains all repositories related in running the uav in simulation.

        # == Enter the Container ==
        
        # enter the docker shell container
        docker-join.bash --name uav-sim-shell

        # == Common Catkin Workspace ==

        # go to the `common` catkin workspace inside the docker container
        cd ~/deploy_ws/src/common

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set uav

        # build the catkin workspace
        catkin build

        # == UAV:Simulation Catkin Workspace ==

        # go to the uav:sim catkin workspace
        cd ~/deploy_ws/src/uav/sim

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set sim

        # build the catkin workspace
        catkin build

        # exit the container
        exit

        # stop the running container
        docker stop uav-sim-shell

## 4. Summary

You should now have a built `SubT` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.

Please go back to [`build-tutorial`](build-tutorial.md#Summary) for summary comments.
