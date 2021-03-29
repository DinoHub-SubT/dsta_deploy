# Basestation Catkin Workspace (Robot)

Setting up the catkin workspace for the basestation workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the basestation catkin workspace.

Assuming you have already setup all your docker containers, follow the instructions below to setup the catkin workspace.

## Catkin Build

        # create the docker shell container
        subt deployer robots.basestation.docker.shell.start

        # clean the previously built workspaces
        subt deployer robots.basestation.catkin.clean

        # catkin build the basestation GUI workspaces
        subt deployer robots.basestation.catkin.build

## Cleanup (optional)

You should remove containers when done with its development (for those that are available).

        # stop the docker container
        subt deployer robots.basestation.docker.shell.stop

        # remove the docker container
        subt deployer robots.basestation.docker.shell.rm

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash -n [container-name]` command will enter a stopped container.

## Summary

You should now have a built `basestation` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
