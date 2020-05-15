# UAV Catkin Workspace

Setting up the catkin workspace for the UAV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UAV catkin workspace.

- There are different docker containers for the different catkin workspaces.

Follow the instructions below to setup the UAV catkin workspace.

## UAV Simulation Workspace

### 1. Access Docker Container

        # ssh into the remote Azure VM (if not already logged in).Change `azure.uav1` to the correct VM name 
        # -- if you are not using Azure, you may skip this step.
        ssh azure.uav1

        # enter the docker shell container on your local laptop host or Azure VM host
        # -- if you are not using Azure, you may skip this step.
        docker-join.bash --name uav-sim-shell

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
        catkin profile set uav

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

### 3. Build UAV Dependencies

The UAV simulation workspace requires manual code changes to thirdparty files.

        # go to px4 firmware repo
        cd ~/deploy_ws/src/uav/sim/Firmware

        # remove cmake error flags
        gedit cmake/px4_add_common_flags.cmake

        # remove line 72:
        # -Werror       <--- remove this line

        # change invalid define boolean value
        gedit Tools/sitl_gazebo/include/gazebo_opticalflow_plugin.h

        # change line 43: with uppercase 'TRUE'
        #define HAS_GYRO TRUE <--- original line

        # To: with lowercase 'true'
        #define HAS_GYRO true <--- modified line

The UAV simulation workspace requires building non-catkin dependencies.

        # go to the uav:sim catkin workspace
        cd ~/deploy_ws/src/uav/sim

        # build the px4 firmware
        cd Firmware
        DONT_RUN=1 make px4_sitl_default gazebo

        # change to root user
        sudo su

        # build mavros
        cd ../mavros/mavros/scripts
        ./install_geographiclib_datasets.sh

        # exit root user
        exit

### 4. Build UAV Simulation Catkin Workspace

The UAV simulation catkin workspace contains all repositories related in running the `uav` in simulation.

The `uav` catkin workspaces sets up default `cmake` options.

        # go to the uav:sim:darpa catkin workspace
        cd ~/deploy_ws/src/uav/sim/darpa/catkin

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set uav

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

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
