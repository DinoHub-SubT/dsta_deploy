# Basestation Catkin Workspace (Localhost)

Setting up the catkin workspace for the basestation workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the basestation catkin workspace.

Assuming you have already setup all your docker containers, follow the instructions below to setup the catkin workspace.

## Catkin Build

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the docker shell on the remote host
        #   - you need to make sure the container is started before building
        ./deployer -s local.basestation.docker.shell

        # clean the previously built workspaces
        ./deployer -s local.basestation.catkin.clean

        # catkin build the basestation GUI workspaces
        ./deployer -s local.basestation.cpu.catkin.gui.build


## Cleanup (optional)

You should remove containers when done with its development (for those that are available).

Automated remove the docker containers:

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # stop the docker container
        ./deployer -s local.basestation.docker.stop

        # remove the docker container
        ./deployer -s local.basestation.docker.rm

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash [container-name]` command will enter a stopped container.

## Summary

You should now have a built `basestation` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
