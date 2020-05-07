# Basestation Catkin Workspace

Setting up the catkin workspace for the Basestation workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the basestation catkin workspace.

Assuming you have already setup all your basestation docker containers, follow the instructions below to setup the basestation catkin workspace.

## 1. Access Docker Container

        # ssh into the remote Azure VM (if not already logged in).Change `azure.basestation` to the correct VM name 
        # -- if you are not using Azure, you may skip this step.
        ssh azure.basestation

        # enter the docker shell container on your local laptop host or Azure VM host
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name gui-shell

        # its okay to ignore the following error if you have not yet built the workspace:
        # -> 'bash: /home/developer/deploy_ws/devel/...: No such file or directory'

## 2. Build Common

The common catkin workspace sets up default `cmake` options.

        # == Common Catkin Workspace ==

        # go to the `common` catkin workspace inside the docker container
        cd ~/deploy_ws/src/common

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set system76-pc

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

## 3. Build Basestation Catkin Workspace

The basestation catkin workspace contains all repositories that are running during `SubT` on the basestation.

        # go to the `basestation` catkin workspace
        cd ~/deploy_ws/src/basestation

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set system76-pc

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

        # exit the container
        exit

        # stop the running container
        docker stop gui-shell

## Cleanup (optional)

You should remove containers when done with its development.

        # stop the running container
        docker stop gui-shell

        # remove the container
        docker rm gui-shell

- The above steps will remove the containers.

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash [container-name]` command will enter a stopped container.

## Summary

You should now have a built `basestation` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
