# Build Basestation With Docker Walkthrough

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

**Basestation Docker Image** 

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # build the basestation docker image
        ./deployer -s desktop.basestation.docker.image

**Cleanup (Required)**

        # Remove any previously created docker containers (optional).
        #   - its okay to ignore error 'Error: No such container' and continue to the next step.
        docker rm -f gui-shell

        # cleanup dangling docker images
        docker rmi -f $(docker images -f "dangling=true" -q)

**Verify Docker Images** 

        # View the docker images built on the localhost
        docker images

        # verify you see the following docker images (in any order):
        #     ->
        #        subt/basestation:gui
        #        subt/basestation:ros


## 2. Creating Docker Containers With Shell Access

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

**Basestation Docker Container Shell Access**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the basestation docker container
        ./deployer -s desktop.basestation.docker.shell

**Verify Docker Containers**

        # view running docker containers
        docker ps

        # verify you see the following docker containers (in any order):
        #   -> gui-shell

**Common Issues**

- Error `Unknown runtime specified nvidia`, please change:

        # open the scenario file
        gedit operations/deploy/scenarios/desktop-basestation.env

        # change the variable 'GPU_ENABLE_TYPE' to be the below:
        export GPU_ENABLE_TYPE="non-gpu"

## 3. Building The Catkin Workspace

**Build Basestation**

The basestation catkin workspace contains all basestation related repositories (example GUI control).

        # == Enter the Container ==

        # enter the docker shell container
        docker-join.bash --name gui-shell

        # == Common Catkin Workspace ==

        # go to the `common` catkin workspace inside the docker container
        cd ~/deploy_ws/src/common

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set system76-pc

        # build the catkin workspace
        catkin build

        # == Basestation Catkin Workspace ==

        # go to the `basestation` catkin workspace
        cd ~/deploy_ws/src/basestation

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set system76-pc

        # build the catkin workspace
        catkin build

        # exit the container
        exit

        # stop the running container
        docker stop gui-shell

## 4. Summary

You should now have a built `SubT` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.

Please go back to [`build-tutorial`](build-tutorial.md#Summary) for summary comments.
