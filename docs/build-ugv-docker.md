# Build UGV With Docker Walkthrough

The `SubT` workspace will be built and will run in docker containers.
    
- Workspace dependencies are installed in docker images.

**Note:** This tutorial will build the docker setup directly on the localhost.


## Quick Start

The quick start will walkthrough the following:

- How to install workspace dependencies in docker images on your localhost.
- How to create docker containers on your localhost.
- How to build the catkin workspaces in the docker containers.

### 1. Building Docker Images

Building docker images on the localhost will install all the workspace dependencies.

**UGV Docker Image**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # build the ugv:planning-pc docker image
        ./deployer -s desktop.ugv.ppc.docker.image

        # build the ugv:nuc docker image
        ./deployer -s desktop.ugv.nuc.docker.image

        # build the ugv:simulation docker image
        ./deployer -s desktop.ugv.sim.docker.image

**Cleanup (Required)**

        # Remove any previously created docker containers (optional)
        #   - its okay to ignore error 'Error: No such container' and continue to the next step.
        docker rm -f ppc-shell nuc-shell sim-shell

        # cleanup dangling docker images
        docker rmi -f $(docker images -f "dangling=true" -q)

**Verify Docker Images** 

        # View the docker images built on the localhost
        docker images

        # verify you see the following docker images (in any order):
        #   ->
        #       subt/ugv:ppc
        #       subt/ugv:nuc
        #       subt/ugv:sim
        #       subt/ugv:ros

## 2. Creating Docker Containers With Shell Access

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

**UGV Docker Container Shell Access**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the ugv:planning-pc docker container
        ./deployer -s desktop.ugv.ppc.docker.shell

        # create the ugv:planning-pc docker container
        ./deployer -s desktop.ugv.nuc.docker.shell

        # create the ugv:simulation docker container
        ./deployer -s desktop.ugv.sim.docker.shell

**Verify Docker Containers** 

        # view running docker containers
        docker ps

        # verify you see the following docker containers (in any order):
        #   -> 
        #       sim-shell
        #       ppc-shell
        #       nuc-shell


## 3. Building The Catkin Workspace

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


**Build UGV: SIMULATION**

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

        # == Common UGV:SIMULATION Workspace ==

        # go to the `nuc` catkin workspace
        cd ~/deploy_ws/src/ugv/sim

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

You should now have a built `SubT` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.

Please go back to [`build-tutorial`](build-tutorial.md#Summary) for summary comments.
