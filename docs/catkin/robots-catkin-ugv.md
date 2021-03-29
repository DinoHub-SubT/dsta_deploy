# UGV Catkin Workspace (Robot)

Setting up the catkin workspace for the UGV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UGV catkin build workspace.

Assuming you have already setup all your docker containers, follow the instructions below to setup the catkin workspace.

## 1. Catkin Build

        # create the docker shell on the remote host
        subt deployer robots.ugv.ugv1.ppc.docker.shell.start
        subt deployer robots.ugv.ugv1.nuc.docker.shell.start
        subt deployer robots.ugv.ugv1.xavier.docker.shell.start

        # clean the previous built workspaces
        subt deployer robots.ugv.ugv1.ppc.catkin.clean
        subt deployer robots.ugv.ugv1.nuc.catkin.clean
        subt deployer robots.ugv.ugv1.xavier.catkin.clean

        # catkin build the UGV workspaces
        subt deployer robots.ugv.ugv1.ppc.catkin.build
        subt deployer robots.ugv.ugv1.nuc.catkin.build
        subt deployer robots.ugv.ugv1.xavier.catkin.build

You can build on all three computers with the following short-hand command:

        # omit the computer tag, to build on all three computers
        subt deployer robots.ugv.ugv1.catkin.clean

- You can remove in-between tags, to shorten commands. Experiment with your options by adding a `-p` to preview your commands.

## Cleanup (optional)

You should remove containers when done with its development.

        # stop the docker container
        subt deployer robots.ugv.ugv1.ppc.docker.shell.stop
        subt deployer robots.ugv.ugv1.nuc.docker.shell.stop
        subt deployer robots.ugv.ugv1.xavier.docker.shell.stop

        # remove the docker container
        subt deployer robots.ugv.ugv1.ppc.docker.shell.rm
        subt deployer robots.ugv.ugv1.nuc.docker.shell.rm
        subt deployer robots.ugv.ugv1.xavier.docker.shell.rm

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash -n [container-name]` command will enter a stopped container.

## Summary

You should now have a built `UGV` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
