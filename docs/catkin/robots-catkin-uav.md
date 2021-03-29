# UAV Catkin Workspace (Robot)

Setting up the catkin workspace for the UAV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UAV catkin workspace.

Assuming you have already setup all your docker containers, follow the instructions below to setup the catkin workspace.

## Catkin Build

        # create the docker shell on the remote host
        #   - you need to make sure the container is started before building
        subt deployer robots.uav.ds1.docker.shell.start

        # (OTIONAL) clean all the workspaces
        subt deployer robots.uav.ds1.catkin.clean

        # build all the workspaces
        subt deployer robots.uav.ds1.catkin.build

## Cleanup (optional)

You should remove containers when done with its development.

        # stop the docker container
        subt deployer robots.uav.ds1.docker.shell.stop

        # remove the docker container
        subt deployer robots.uav.ds1.docker.shell.rm

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash -n [container-name]` command will enter a stopped container.

## Summary

You should now have a built `UAV` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.

