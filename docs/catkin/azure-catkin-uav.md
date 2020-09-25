# UAV Catkin Workspace

Setting up the catkin workspace for the UAV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UAV catkin workspace.

- There are different docker containers for the different catkin workspaces.

Follow these instructions below, **on the localhost**, not on the Azure remote VM, to setup the UAV catkin workspace.

## Transfer

You can transfer changes from your localhost to the remote:

        # uav transfer.to command
        subt deployer azure.uav.uav1.transfer.to

If you find the `transfer.to` is too slow. then try this command:

        subt deployer azure.uav.uav1.skel_t.to

You can edit the transfer options: `deploy_rsync_opts` in `operations/deploy/scenarios/.uav.env`

- This option tells the deployer to **exclude** files during the transfer. You may change the files that get excluded.
- **Example change:** adding `--exclude=src/.git`, will reduce the time for the transfer, but you wont see any git changes reflected on the remote.

## Catkin Build

Follow this step, **on the localhost**, not on the Azure remote VM.

        # start the container
        subt deployer azure.uav.uav1.docker.shell

        # clean the previous built workspaces
        subt deployer azure.uav.uav1.catkin.clean

        # build the PX4 firmware
        subt deployer azure.uav.uav1.catkin.px4

        # catkin build the 'core' UGV workspaces
        subt deployer azure.uav.uav1.catkin.core.build

- Please change the robot name `uav1` to whichever Azure robot VM you are building on.

## Cleanup (optional)

You should remove containers when done with its development.

Automated remove the docker containers:

        # stop the docker container
        subt deployer azure.uav.uav1.docker.stop

        # remove the docker container
        subt deployer azure.uav.uav1.docker.rm

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
