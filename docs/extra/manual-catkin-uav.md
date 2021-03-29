# UAV Catkin Workspace

Setting up the catkin workspace for the UAV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UAV catkin workspace.

- There are different docker containers for the different catkin workspaces.

Follow the instructions below to setup the UAV catkin workspace.

## UAV Simulation Catkin Workspaces

### 1. Apply UAV Firmware Patch

Follow this step, **on the localhost**, not on the Azure remote VM.

You only need to apply the firmware patch once, on a fresh clone:

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # apply firmware patch
        ./deployer -s local.uav.patch

- This will apply an update to the `cmake` files found in `~/deploy_ws/src/uav/core/Firmware`

This is a temporary fix, a permanent solution will be investigated.

### 2. Access Docker Container

        # ssh into the remote Azure VM (if not already logged in).Change `azure.uav1` to the correct VM name
        # -- if you are not using Azure, you may skip this step.
        ssh azure.uav1

        # enter the docker shell container on your local laptop host or Azure VM host
        # -- if you are not using Azure, you may skip this step.
        docker-join.bash --name uav-sim-shell

        # its okay to ignore the following error if you have not yet built the workspace:
        # -> 'bash: /home/developer/deploy_ws/devel/...: No such file or directory'

### 3. Build Common

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

### 4. Build UAV Dependencies

The UAV simulation workspace requires building non-catkin thirdparty dependencies.

        # go to px4 firmware repo
        cd ~/deploy_ws/src/uav/core/Firmware

        # build the px4 firmware
        DONT_RUN=1 make px4_sitl_default gazebo

### 5. Build UAV Catkin Workspace

The UAV simulation catkin workspace contains all repositories related in running the `uav` in simulation.

The `uav` catkin workspaces sets up default `cmake` options.

        # go to the uav catkin workspace
        cd ~/deploy_ws/src/uav/

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set uav-sim

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

### 6. Build Simulation Catkin Workspace

The UAV simulation catkin workspace contains all repositories related in running the `uav` in simulation.

The `uav` catkin workspaces sets up default `cmake` options.

        # go to the simulation:darpa catkin workspace
        cd ~/deploy_ws/src/simulation/darpa/catkin

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set uav

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

        # go to the simulation catkin workspace
        cd ~/deploy_ws/src/simulation

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set uav

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

### 7. Build SubT Launch Catkin Workspace

The subt launch catkin workspace contains a centralized top-level launch.

        # go to the `subt_launch` catkin workspace
        cd ~/deploy_ws/src/subt_launch

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set uav-simulation

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

- The `docker-join.bash -n [container-name]` command will enter a stopped container.

