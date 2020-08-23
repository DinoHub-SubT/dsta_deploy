# UGV Catkin Workspace (Robot)

Setting up the catkin workspace for the UGV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UGV catkin build workspace.

Assuming you have already setup all your docker containers, follow the instructions below to setup the catkin workspace.

## 1. Catkin Build

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the docker shell on the remote host
        #   - you need to make sure the container is started before building
        ./deployer -s azure.ugv1.docker.shell

        # clean the previous built workspaces
        ./deployer -s azure.ugv1.catkin.clean

        # catkin build the UGV workspaces
        ./deployer -s azure.ugv1.catkin.build

        # (optional), building on specific computers
        #   - similar commands exist for *.clean
        ./deployer -s azure.ugv1.ppc.catkin.build
        ./deployer -s azure.ugv1.nuc.catkin.build
        ./deployer -s azure.ugv1.xavier.catkin.build

## Cleanup (optional)

You should remove containers when done with its development.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # stop the docker container
        ./deployer -s azure.ugv1.docker.stop

        # remove the docker container
        ./deployer -s azure.ugv1.docker.rm

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash [container-name]` command will enter a stopped container.

## Summary

You should now have a built `UGV` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
