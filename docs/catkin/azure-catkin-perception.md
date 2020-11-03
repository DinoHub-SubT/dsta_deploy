# Perception Catkin Workspace

Setting up the catkin workspace for the Perception workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the perception catkin workspace.

Assuming you have already setup all your perception docker containers, follow the instructions below to setup the perception catkin workspace.

## Azure CPU VM

If you **have not enabled** the GPU on the remote perception Azure VM, then follow these instructions.

Follow these steps, **on the localhost**, not on the Azure remote VM.

### 1. Catkin Build

        # create the docker shell on the remote host
        subt deployer azure.perception.perception1.cpu.docker.shell

        # clean the previous built workspaces
        subt deployer azure.perception.perception1.cpu.catkin.clean

        # catkin build the UGV workspaces
        subt deployer azure.perception.perception1.cpu.catkin.build

- Please change the robot name `perception1` to whichever Azure robot VM you are building on.

## Azure GPU VM

If you **have enabled** the GPU on the remote basestation Azure VM, then follow these instructions.

Follow these steps, **on the localhost**, not on the Azure remote VM.

### 1. Catkin Build

        # create the docker shell on the remote host
        subt deployer azure.perception.perception1.gpu.docker.shell

        # clean the previous built workspaces
        subt deployer azure.perception.perception1.gpu.catkin.clean

        # catkin build the UGV workspaces
        subt deployer azure.perception.perception1.gpu.catkin.build

- Please change the robot name `perception1` to whichever Azure robot VM you are building on.

## Cleanup (optional)

You should remove containers when done with its development.

Automated remove the docker containers:

        # stop the docker container
        subt deployer azure.perception.perception1.docker.stop

        # remove the docker container
        subt deployer azure.perception.perception1.docker.rm

Or manually remove the docker containers:

        # stop the running containers
        docker stop perception-cpu-shell perception-gpu-shell

        # remove the containers
        docker rm perception-cpu-shell perception-gpu-shell

- The above steps will remove the containers.

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash [container-name]` command will enter a stopped container.

## Summary

You should now have a built `perception` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.

## Helpful Tips

You can transfer changes from your localhost to the remote:

        # perception transfer.to command
        subt deployer azure.perception.perception1.transfer.to

If you find the `transfer.to` is too slow or missing files during a transfer, you can find the the `transfer.to` options in the file:

        operations/deploy/scenarios/.perception.env

You can edit the option: `deploy_rsync_opts`

- This option tells the deployer to **exclude** files during the transfer. You may change the files that get excluded.
- **Example change:** adding `--exclude=src/.git`, will reduce the time for the transfer, but you wont see any git changes reflected on the remote.
