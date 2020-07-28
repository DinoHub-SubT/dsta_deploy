# UGV Catkin Workspace

Setting up the catkin workspace for the UGV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UGV catkin build workspace.

- There are different docker containers for the different catkin workspaces.

Follow the instructions below to setup the UGV catkin workspace.

## UGV Simulation (Planning-PC, NUC) Catkin Workspaces

Follow these steps, **on the localhost**, not on the Azure remote VM.

### 1. Catkin Build

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the docker shell on the remote host
        ./deployer -r azure.ugv1.docker.shell

        # clean the previous built workspaces
        ./deployer -r azure.ugv1.catkin.clean

        # catkin build the UGV workspaces
        ./deployer -r azure.ugv1.catkin.build

- Please change the robot name `ugv1` to whichever Azure robot VM you are building on.

## Cleanup (optional)

You should remove containers when done with its development.

Automated remove the docker containers:

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # stop the docker container
        ./deployer -r azure.ugv1.docker.stop

        # remove the docker container
        ./deployer -r azure.ugv1.docker.remove

Or manually remove the docker containers:

        # stop the running container
        docker stop ppc-shell

        # remove the container
        docker rm ppc-shell

        # stop the running container
        docker stop nuc-shell

        # remove the container
        docker rm nuc-shell

        # stop the running container
        docker stop ugv-sim-shell

        # remove the container
        docker rm ugv-sim-shell

- The above steps will remove the containers.

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash [container-name]` command will enter a stopped container.

## Summary

You should now have a built `UGV` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.

## Helpful Tips

You can transfer changes from your localhost to the remote:

        # ugv transfer.to command
        ./deployer -r azure.ugv1.transfer.to

If you find the `transfer.to` is too slow or missing files during a transfer, you can find the the `transfer.to` options in the file:

        operations/deploy/scenarios/.ugv.env

You can edit the option: `deploy_rsync_opts`

- This option tells the deployer to **exclude** files during the transfer. You may change the files that get excluded.
- **Example change:** adding `--exclude=src/.git`, will reduce the time for the transfer, but you wont see any git changes reflected on the remote.