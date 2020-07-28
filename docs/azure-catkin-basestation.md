# Basestation Catkin Workspace

Setting up the catkin workspace for the Basestation workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the basestation catkin workspace.

Assuming you have already setup all your basestation docker containers, follow the instructions below to setup the basestation catkin workspace.

## Azure CPU VM

If you **have not enabled** the GPU on the remote basestation Azure VM, then follow these instructions.

Follow these steps, **on the localhost**, not on the Azure remote VM.

### 1. Catkin Build

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the docker shell on the remote host
        ./deployer -r azure.basestation.cpu.docker.shell

        # clean the previous built workspaces
        ./deployer -r azure.basestation.cpu.catkin.gui.clean

        # catkin build the basestation GUI workspaces
        ./deployer -r azure.basestation.cpu.catkin.gui.build


## Azure GPU VM

If you **have enabled** the GPU on the remote basestation Azure VM, then follow these instructions.

Follow these steps, **on the localhost**, not on the Azure remote VM.

### 1. Catkin Build

You want to build both the `cpu` and `gpu` catkin workspaces on the GPU VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # clean the previous built workspaces
        ./deployer -r azure.basestation.cpu.catkin.gui.clean
        ./deployer -r azure.basestation.gpu.catkin.perception.clean

        # create the docker shell on the remote host
        ./deployer -r azure.basestation.cpu.docker.shell
        ./deployer -r azure.basestation.gpu.docker.shell

        # catkin build the basestation GUI workspaces
        ./deployer -r azure.basestation.cpu.catkin.gui.build

        # catkin build the basestation perception workspaces
        ./deployer -r azure.basestation.gpu.catkin.perception.build


## Cleanup (optional)

You should remove containers when done with its development.

Automated remove the docker containers:

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # stop the docker container
        ./deployer -r azure.basestation.docker.stop

        # remove the docker container
        ./deployer -r azure.basestation.docker.remove

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

## Helpful Tips

You can transfer changes from your localhost to the remote:

        # basestation transfer.to command
        ./deployer -r azure.basestation.transfer.to

If you find the `transfer.to` is too slow or missing files during a transfer, you can find the the `transfer.to` options in the file:

        operations/deploy/scenarios/.basestation.env

You can edit the option: `deploy_rsync_opts`

- This option tells the deployer to **exclude** files during the transfer. You may change the files that get excluded.
- **Example change:** adding `--exclude=src/.git`, will reduce the time for the transfer, but you wont see any git changes reflected on the remote.
