# UAV Catkin Workspace

Setting up the catkin workspace for the UAV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UAV catkin workspace.

Assuming you have already setup all your docker containers, follow the instructions below to setup the catkin workspace.

## Prerequisites

If you have Intel Integrated Graphics, please follow the below instructions with the `cpu` tag.
If you have NVIDIA Graphics, please follow the below instructions, **but substitute the `cpu` tag with the `gpu` tag**.

If you have neither, please notify the maintainer. You will need one of those two options to run.

## Catkin Build

        # PLEASE NOTICE
        #   - user either the cpu or gpu deployer commands. NOT BOTH!
        #   - if you're computer has an NVIDIA GPU, then use the gpu shell. Otherwise use the cpu shell.
        #   - each of the below commands has a cpu or gpu commands, please choose the one for your setup.

        # cpu shell

        # start the container
        subt deployer local.uav.uav1.core.docker.shell.start

        # clean the previous built workspaces
        subt deployer local.uav.uav1.core.catkin.clean

        # build the uav's core workspace
        subt deployer local.uav.uav1.core.catkin.build

## Cleanup (optional)

You should remove containers when done with its development (for those that are available).

        # stop the docker container
        subt deployer local.uav.uav1.core.docker.shell.stop

        # remove the docker container
        subt deployer local.uav.uav1.core.docker.shell.rm

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash -n [container-name]` command will enter a stopped container.

## Summary

You should now have a built `UAV` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
