# UAV Catkin Workspace

Setting up the catkin workspace for the UAV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UAV catkin workspace.

- There are different docker containers for the different catkin workspaces.

Follow the instructions below to setup the UAV catkin workspace.

## UAV Simulation Catkin Workspaces

Follow these steps, **on the localhost**, not on the Azure remote VM.

### 1. Apply UAV Firmware Patch

You only need to apply the firmware patch once, on a fresh clone:

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # apply firmware patch
        ./deployer -s local.uav.patch

- This will apply an update to the `cmake` files found in `~/deploy_ws/src/uav/core/Firmware`

This is a temporary fix, a permanent solution will be investigated.

### 2. Catkin Build

Follow this step, **on the localhost**, not on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the docker shell on the remote host
        ./deployer -r azure.uav1.docker.shell

        # clean the previous built workspaces
        ./deployer -r azure.uav1.catkin.clean

        # build the PX4 firmware
        ./deployer -r azure.uav1.px4_firmware

        # catkin build the UGV workspaces
        ./deployer -r azure.uav1.catkin.build

- Please change the robot name `uav1` to whichever Azure robot VM you are building on.

## Cleanup (optional)

You should remove containers when done with its development.

Automated remove the docker containers:

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # stop the docker container
        ./deployer -r azure.uav1.docker.stop

        # remove the docker container
        ./deployer -r azure.uav1.docker.remove

Or manually remove the docker containers:

        # stop the running container
        docker stop uav-sim-shell

        # remove the running container
        docker rm uav-sim-shell

- The above steps will remove the containers.

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash [container-name]` command will enter a stopped container.

## Summary

You should now have a built `UAV` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.

## Helpful Tips

You can transfer changes from your localhost to the remote:

        # uav transfer.to command
        ./deployer -r azure.uav1.transfer.to

If you find the `transfer.to` is too slow or missing files during a transfer, you can find the the `transfer.to` options in the file:

        operations/deploy/scenarios/.uav.env

You can edit the option: `deploy_rsync_opts`

- This option tells the deployer to **exclude** files during the transfer. You may change the files that get excluded.
- **Example change:** adding `--exclude=src/.git`, will reduce the time for the transfer, but you wont see any git changes reflected on the remote.