# UGV Catkin Workspace (Robot)

Setting up the catkin workspace for the ugv workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the ugv catkin workspace.

Assuming you have already setup all your docker containers, follow the instructions below to setup the catkin workspace.

## Catkin Build

```text
# create the docker shell on the remote host
#   - you need to make sure the container is started before building
subt deployer robots.ugv.ugv1.docker.shell.start

# (optional) run on a specific ugv robot computer
subt deployer robots.ugv.ugv1.ppc.docker.shell.start
subt deployer robots.ugv.ugv1.nuc.docker.shell.start
subt deployer robots.ugv.ugv1.xavier.docker.shell.start

# (OTIONAL) clean all the workspaces
subt deployer robots.ugv.ugv1.catkin.clean

# (optional) run on a specific ugv robot computer
subt deployer robots.ugv.ugv1.ppc.catkin.clean
subt deployer robots.ugv.ugv1.nuc.catkin.clean
subt deployer robots.ugv.ugv1.xavier.catkin.clean

# build all the workspaces
subt deployer robots.ugv.ugv1.catkin.build

# (optional) run on a specific ugv robot computer
subt deployer robots.ugv.ugv1.ppc.catkin.build
subt deployer robots.ugv.ugv1.nuc.catkin.build
subt deployer robots.ugv.ugv1.xavier.catkin.build
```

## Cleanup (optional)

You should remove containers when done with its development.

        # stop the docker container
        subt deployer robots.ugv.ugv1.docker.shell.stop

        # remove the docker container
        subt deployer robots.ugv.ugv1.docker.shell.rm

- When you continue with development, you will need to re-create the docker containers again.

- You can just stop the docker containers rather than completely removing them, to avoid re-creating them all the time.

- The `docker-join.bash -n [container-name]` command will enter a stopped container.

## Summary

You should now have a built `ugv` workspace.

- Please notify the maintainer if any of the tutorial steps did not succeed.
