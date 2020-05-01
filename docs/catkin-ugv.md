# UGV Catkin Workspace

Setting up the catkin workspace for the UGV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UGV catkin build workspace.

- There are different docker containers for the different catkin workspaces.

Follow the instructions below to setup the UGV catkin workspace.

## UGV Planning-PC Catkin Workspace

### 1. Access Docker Container (optional)

**If you are not using docker containers, you may skip this step.**

**If you are using an Azure VM, remember to ssh into the VM first.**

        # enter the docker shell container on your local laptop host or Azure VM host
        #   -- its okay to ignore the error if you have not yet built the workspace: error is: 'bash: /home/developer/deploy_ws/devel/...: No such file or directory'
        docker-join.bash --name ppc-shell

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

### 3. Build UGV Planning-PC Catkin Workspace

The UGV planning-pc catkin workspace contains all repositories that are running on the robot, on the planning-pc.

The `ugv:planning-pc` catkin workspace sets up default `cmake` options.

        # go to the `ppc` catkin workspace
        cd ~/deploy_ws/src/ugv/planning-pc

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set ugv

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

        # exit the container
        exit

## UGV NUC Catkin Workspace

### 1. Access Docker Container (optional)

**If you are not using docker containers, you may skip this step.**

**If you are using an Azure VM, remember to ssh into the VM first.**

        # enter the docker shell container on your local laptop host or Azure VM host
        #   -- its okay to ignore the error if you have not yet built the workspace: error is: 'bash: /home/developer/deploy_ws/devel/...: No such file or directory'
        docker-join.bash --name nuc-shell

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

### 3. Build UGV NUC Catkin Workspace

The UGV nuc catkin workspace contains all repositories that are running on the robot, on the nuc.

The `ugv:nuc` catkin workspace sets up default `cmake` options.

        # go to the `nuc` catkin workspace
        cd ~/deploy_ws/src/ugv/nuc

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set ugv

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

        # exit the container
        exit


## UGV Simulation Catkin Workspace

### 1. Access Docker Container (optional)

**If you are not using docker containers, you may skip this step.**

**If you are using an Azure VM, remember to ssh into the VM first.**

        # enter the docker shell container on your local laptop host or Azure VM host
        #   -- its okay to ignore the error if you have not yet built the workspace: error is: 'bash: /home/developer/deploy_ws/devel/...: No such file or directory'
        docker-join.bash --name sim-shell

### 2. Build Common

The common catkin workspace sets up default `cmake` options.

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

### 3. Build UGV Simulation Catkin Workspace

The UGV simulation catkin workspace contains all repositories related in running the `ugv` in simulation.

The `ugv:ugv` catkin workspace sets up default `cmake` options.

        # go to the `sim:darpa` catkin workspace
        cd ~/deploy_ws/src/ugv/sim/darpa/catkin/

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set ugv

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

        # go to the `sim` catkin workspace
        cd ~/deploy_ws/src/ugv/sim

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set ugv

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

        # exit the container
        exit

* * *

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
        docker stop sim-shell

        # remove the container
        docker rm sim-shell

- The above steps will remove the containers.

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash [container-name]` command will enter a stopped container.

## Summary

You should now have a built `UGV` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
