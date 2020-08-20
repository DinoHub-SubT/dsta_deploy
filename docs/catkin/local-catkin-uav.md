# UAV Catkin Workspace

Setting up the catkin workspace for the UAV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UAV catkin workspace.

Assuming you have already setup all your docker containers, follow the instructions below to setup the catkin workspace.

## Catkin Build

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # PLEASE NOTICE
        #   - user either the cpu or gpu deployer commands. NOT BOTH!
        #   - if you're computer has an NVIDIA GPU, then use the gpu shell. Otherwise use the cpu shell.
        #   - each of the below commands has a cpu or gpu commands, please choose the one for your setup.

        # create the docker shell on the remote host
        # cpu
        ./deployer -s local.uav.cpu.docker.shell
        # gpu
        ./deployer -s local.uav.gpu.docker.shell

        # clean the previous built workspaces
        ./deployer -s local.uav.cpu.catkin.clean

        # build the PX4 firmware
        ./deployer -s local.uav.cpu.px4

        # catkin build the UGV workspaces
        ./deployer -s local.uav.cpu.catkin.build

## Cleanup (optional)

You should remove containers when done with its development (for those that are available).

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # stop the docker container
        ./deployer -s local.uav.docker.stop

        # remove the docker container
        ./deployer -s local.uav.docker.rm

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash [container-name]` command will enter a stopped container.

## Summary

You should now have a built `UAV` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
