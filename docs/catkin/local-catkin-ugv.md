# UGV Catkin Workspace

Setting up the catkin workspace for the ugv workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the ugv catkin workspace.

Assuming you have already setup all your docker containers, follow the instructions below to setup the catkin workspace.

## Prerequisites

## Catkin Build

```text
# start the container
subt deployer local.ugv.ugv1.core.docker.shell.start

# clean the previous built workspaces
subt deployer local.ugv.ugv1.core.catkin.clean

# build the ugv's core workspace
subt deployer local.ugv.ugv1.core.catkin.build
```

## Cleanup (optional)

You should remove containers when done with its development (for those that are available).

```text
# stop the docker container
subt deployer local.ugv.ugv1.core.docker.shell.stop

# remove the docker container
subt deployer local.ugv.ugv1.core.docker.shell.rm
```

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash -n [container-name]` command will enter a stopped container.

## Summary

You should now have a built `ugv` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
