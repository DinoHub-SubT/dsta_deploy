# Docker Deployment Example Walkthrough

Everything will build and run in docker containers.

- **All dependencies must be installed in docker images**

This tutorial will outline how to setup every workspace, please  only follow instructions that matches your chosen setup.

* * *

## Quick Start

**Basic Level**

- Explains how to install workspace dependencies as docker images, create docker containers and build catkin workspaces.
- This level is the *recommended method to use* because of the simple for repository access and debugging in docker containers.

### 1. Building Docker Images

You only need to build docker images whenever you make changes to dockerfiles found in: `operations/deploy/docker/dockerfiles`

  - You should make changes to dockerfiles when you want to add workspace thirdparty dependencies.
  - If you install a dependency in the container directly, please remember to put it in the dockerfile and rebuild the docker image.


**Basestation Docker Image** 

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # build the basestation docker image
        ./deployer -s desktop.basestation.docker.image

**UGV Docker Image**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # build the ugv:planning-pc docker image
        ./deployer -s desktop.ugv.ppc.docker.image

        # build the ugv:nuc docker image
        ./deployer -s desktop.ugv.nuc.docker.image

**UAV Docker Image**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # build the ugv planning-pc docker image
        ./deployer -s desktop.uav.docker.image

**Cleanup (Required)**

        # Remove any previously created docker containers (optional)
        docker rm -f uav-sim-shell gui-shell ppc-shell nuc-shell

        # cleanup dangling docker images
        docker rmi -f $(docker images -f "dangling=true" -q)


## 2. Creating Docker Shell Access Containers

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything you would do normally do direclty on the host, but instead inside the docker container.

**Basestation Docker Container Shell Access**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the basestation docker container
        ./deployer -s desktop.basestation.docker.shell

        # view running docker containers
        docker ps

        # verify you see the docker container from the output in `docker ps`:
        #   -> gui-shell

**UGV Docker Container Shell Access**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the ugv:planning-pc docker container
        ./deployer -s desktop.ugv.ppc.docker.shell

        # view running docker containers
        docker ps

        # verify you see the docker container from the output in `docker ps`:
        #   -> ppc-shell

        # create the ugv:planning-pc docker container
        ./deployer -s desktop.ugv.nuc.docker.shell

        # view running docker containers
        docker ps

        # verify you see the docker container from the output in `docker ps`:
        #   -> nuc-shell

**UAV Docker Container Shell Access**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the ugv:planning-pc docker container
        ./deployer -s desktop.uav.docker.shell

        # view running docker containers
        docker ps

        # verify you see the docker container from the output in `docker ps`:
        #   -> uav-shell


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

**Build UGV: Planning-PC**

The ugv planning-pc catkin workspace contains all repositories related in running the ugv's planning-pc in simulation.

        # == Enter the Container ==

        # enter the docker shell container
        docker-join.bash --name ppc-shell

        # == Common Catkin Workspace ==

        # go to the `common` catkin workspace inside the docker container
        cd ~/deploy_ws/src/common

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set ugv

        # build the catkin workspace
        catkin build

        # == Common UGV:Planning-PC Workspace ==

        # go to the `ppc` catkin workspace
        cd ~/deploy_ws/src/ugv/planning-pc

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set ugv

        # build the catkin workspace
        catkin build

        # exit the container
        exit

        # stop the running container
        docker stop ppc-shell

**Build UGV: NUC**

The ugv nuc catkin workspace contains all repositories related in running the ugv's nuc in simulation.

        # == Common Catkin Workspace ==

        # enter the docker shell container
        docker-join.bash --name nuc-shell

        # go to the `common` catkin workspace inside the docker container
        cd ~/deploy_ws/src/common

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set ugv

        # build the catkin workspace
        catkin build

        # == Common UGV:NUC Workspace ==

        # go to the `nuc` catkin workspace
        cd ~/deploy_ws/src/ugv/nuc

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set ugv

        # build the catkin workspace
        catkin build

        # exit the container
        exit

        # stop the running container
        docker stop nuc-shell

## 4. Summary

You should now have a built SubT workspace.

You should become more familiar with operational tools for different types of operations:

**Template: Creating Docker Images**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # Create the docker image
        ./deployer -s [deployment host].[(optional) robot].[(optional) computer].docker.image

**Template: Creating Docker Shell Access Containers**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # Create the docker shell container
        ./deployer -s [deployment host].[(optional) robot].[(optional) computer].docker.shell

**Template: Building the catkin workspace**

        # enter the docker shell container
        docker-join.bash --name gui-shell

        # go to the workspace path
        cd ~/deploy_ws/src/path/to/workspace

        # view the catkin profiles
        catkin profile list

        # select the catkin profile
        catkin profile set [profile name]

To learn more about the available options, please use the `--preview` or `-p` command as example:

        # preview ugv options when deploying on the dekstop
        ./deployer -s desktop.ugv -p

To learn more about the available options actually do, please use the `--verbose` or `-v` option with the `preview` option

        # preview, verbose the ugv options when deploying on the dekstop
        ./deployer -s desktop.ugv -p -v
