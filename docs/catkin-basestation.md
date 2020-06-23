# Basestation Catkin Workspace

Setting up the catkin workspace for the Basestation workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the basestation catkin workspace.

Assuming you have already setup all your basestation docker containers, follow the instructions below to setup the basestation catkin workspace.

## Azure CPU VM

If you **have not enabled** the GPU on the remote basestation Azure VM, then follow these instructions.

### Automated Catkin Build

Follow this step, **on the localhost**, not on the Azure remote VM. These steps will create the docker image on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the docker shell on the remote host
        ./deployer -r azure.basestation.cpu.docker.shell

        # clean the previous built workspaces
        ./deployer -r azure.basestation.cpu.catkin.gui.clean

        # catkin build the basestation GUI workspaces
        ./deployer -r azure.basestation.cpu.catkin.gui.build

### Manual Catkin Build

#### 1. Access Docker Container

        # ssh into the remote Azure VM (if not already logged in).Change `azure.basestation` to the correct VM name
        # -- if you are not using Azure, you may skip this step.
        ssh azure.basestation

        # enter the docker shell container on your local laptop host or Azure VM host
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name basestation-cpu-shell

        # its okay to ignore the following error if you have not yet built the workspace:
        # -> 'bash: /home/developer/deploy_ws/devel/...: No such file or directory'

#### 2. Build Common

The common catkin workspace sets up default `cmake` options.

        # == Common Catkin Workspace ==

        # go to the `common` catkin workspace inside the docker container
        cd ~/deploy_ws/src/common

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set basestation

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

#### 3. Build Basestation Catkin Workspace

The basestation catkin workspace contains all repositories that are running during `SubT` on the basestation.

        # go to the `basestation` catkin workspace
        cd ~/deploy_ws/src/basestation

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set basestation

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

#### 4. Build SubT Launch Catkin Workspace

The subt launch catkin workspace contains a centralized top-level launch.

        # go to the `subt_launch` catkin workspace
        cd ~/deploy_ws/src/subt_launch

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set basestation

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

        # exit the container
        exit

* * *

## Azure GPU VM

If you **have enabled** the GPU on the remote basestation Azure VM, then follow these instructions.

### Automated Catkin Build

Follow this step, **on the localhost**, not on the Azure remote VM. These steps will create the docker image on the Azure remote VM.

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

* * *

## Cleanup (optional)

You should remove containers when done with its development.

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
