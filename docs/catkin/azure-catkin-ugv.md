# UGV Catkin Workspace

Setting up the catkin workspace for the UGV workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the UGV catkin build workspace.

- There are different docker containers for the different catkin workspaces.

Follow these instructions below, **on the localhost**, not on the Azure remote VM, to setup the UGV catkin workspace.

## Transfer

You can transfer changes from your localhost to the remote:

        # ugv transfer.to command
        subt deployer azure.ugv.ugv1.transfer.to

If you find the `transfer.to` is too slow. then try this command:

        subt deployer azure.ugv.ugv1.skel_t.to

- `skel_t.to` will not copy your `.git` so your VM's git status will not be synchronized.
- A suggested practice would be to just use `git` on your localhost and just transfer code to VMs.

You can edit the transfer options: `deploy_rsync_opts` in `operations/scenarios/transfer/ugv.env`

- This option tells the deployer to **exclude** files during the transfer. You may change the files that get excluded.
- **Example change:** adding `--exclude=src/.git`, will reduce the time for the transfer, but you wont see any git changes reflected on the remote.

## Catkin Build

        # create the docker shell on the remote host
        subt deployer azure.ugv.ugv1.core.docker.shell.start

        # clean the previous built workspaces
        subt deployer azure.ugv.ugv1.core.catkin.clean

        # catkin build the UGV simulation workspaces
        subt deployer azure.ugv.ugv1.core.catkin.build

- Please change the robot name `ugv1` to whichever Azure robot VM you are building on.

## Cleanup (optional)

You should remove containers when done with its development.

Automated remove the docker containers:

        # stop the docker container
        subt deployer azure.ugv.ugv1.core.docker.shell.stop

        # remove the docker container
        subt deployer azure.ugv.ugv1.core.docker.shell.rm

- The above steps will remove the containers.

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash -n [container-name]` command will enter a stopped container.

## Summary

You should now have a built `UGV` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
