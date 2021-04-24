# Basestation Catkin Workspace

Setting up the catkin workspace for the Basestation workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the basestation catkin workspace.

Assuming you have already setup all your basestation docker containers, follow the instructions below to setup the basestation catkin workspace.

## Transfer

You can transfer changes from your localhost to the remote:

```text
subt deployer azure.basestation.transfer.to
```

If you find the `transfer.to` is too slow. then try this command:

```text
subt deployer azure.basestation.skel_t.to
```

You can edit the transfer options: `deploy_rsync_opts` in ``operations/scenarios/transfer/basestation.env`

- This option tells the deployer to **exclude** files during the transfer. You may change the files that get excluded.
- **Example change:** adding `--exclude=src/.git`, will reduce the time for the transfer, but you wont see any git changes reflected on the remote.

## Catkin Build

```text
# create the docker shell on the remote host
#   - you need to make sure the container is started before building
subt deployer azure.basestation.docker.shell.start

# clean the previously built workspaces
subt deployer azure.basestation.catkin.clean

# catkin build the basestation GUI workspaces
subt deployer azure.basestation.catkin.build
```

## Cleanup (optional)

You should remove containers when done with its development (for those that are available).

Automated remove the docker containers:

```text
# stop the docker container
subt deployer azure.basestation.docker.shell.stop

# remove the docker container
subt deployer azure.basestation.docker.shell.rm
```

- The above steps will remove the containers.

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash -n [container-name]` command will enter a stopped container.

## Summary

You should now have a built `basestation` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
