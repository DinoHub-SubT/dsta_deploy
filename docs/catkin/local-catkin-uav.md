# UAV Catkin Workspace

Setting up the catkin workspace for the UAV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UAV catkin workspace.

Assuming you have already setup all your docker containers, follow the instructions below to setup the catkin workspace.

## Catkin Build

        # PLEASE NOTICE
        #   - user either the cpu or gpu deployer commands. NOT BOTH!
        #   - if you're computer has an NVIDIA GPU, then use the gpu shell. Otherwise use the cpu shell.
        #   - each of the below commands has a cpu or gpu commands, please choose the one for your setup.

        # cpu shell

        # start the container
        subt deployer local.uav.cpu.docker.shell

        # clean the previous built workspaces
        subt deployer local.uav.cpu.catkin.clean

        # build the PX4 firmware
        subt deployer local.uav.cpu.catkin.px4

        # catkin build the UGV workspaces
        subt deployer local.uav.cpu.catkin.build

## Cleanup (optional)

You should remove containers when done with its development (for those that are available).

        # stop the docker container
        subt deployer local.uav.cpu.docker.stop

        # remove the docker container
        subt deployer local.uav.cpu.docker.rm

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash [container-name]` command will enter a stopped container.

## Summary

You should now have a built `UAV` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
