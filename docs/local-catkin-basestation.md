# Basestation Catkin Workspace

Setting up the catkin workspace for the Basestation workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the basestation catkin workspace.

Assuming you have already setup all your basestation docker containers, follow the instructions below to setup the basestation catkin workspace.

## Azure CPU VM

If you **have not enabled** the GPU on the remote basestation Azure VM, then follow these instructions.

### 1. Catkin Build

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the docker shell on the remote host
        ./deployer -s local.basestation.cpu.docker.shell

        # clean the previous built workspaces
        ./deployer -s local.basestation.cpu.catkin.gui.clean

        # catkin build the basestation GUI workspaces
        ./deployer -s local.basestation.cpu.catkin.gui.build

## Azure GPU VM

If you **have enabled** the GPU on the remote basestation Azure VM, then follow these instructions.

### 1. Catkin Build

You want to build both the `cpu` and `gpu` catkin workspaces on the GPU VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # clean the previous built workspaces
        ./deployer -s local.basestation.cpu.catkin.gui.clean
        ./deployer -s local.basestation.gpu.catkin.perception.clean

        # create the docker shell on the remote host
        ./deployer -s local.basestation.cpu.docker.shell
        ./deployer -s local.basestation.gpu.docker.shell

        # catkin build the basestation GUI workspaces
        ./deployer -s local.basestation.cpu.catkin.gui.build

        # catkin build the basestation perception workspaces
        ./deployer -s local.basestation.gpu.catkin.perception.build

## Cleanup (optional)

You should remove containers when done with its development (for those that are available).

Automated remove the docker containers:

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # stop the docker container
        ./deployer -s local.basestation.docker.stop

        # remove the docker container
        ./deployer -s local.basestation.docker.remove

Or manually remove the docker containers:

        # stop the running container
        docker stop basestation-cpu-shell basestation-gpu-shell

        # remove the container
        docker rm basestation-cpu-shell basestation-gpu-shell

- The above steps will remove the containers.

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash [container-name]` command will enter a stopped container.

## Summary

You should now have a built `basestation` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.

