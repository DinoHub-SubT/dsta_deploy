# Perception Catkin Workspace

Setting up the catkin workspace for the Perception workspaces requires using the `catkin` tool.

- There are multiple catkin workspaces that get extended *or linked* in order to fully setup the perception catkin workspace.

Assuming you have already setup all your perception docker containers, follow the instructions below to setup the perception catkin workspace.

## UAV Perception Catkin Workspaces

### Automated Catkin Build, Azure GPU VM

Follow this step, **on the localhost**, not on the Azure remote VM. These steps will create the docker image on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the docker shell on the remote host
        ./deployer -r azure.perception1.gpu.docker.shell

        # clean the previous built workspaces
        ./deployer -r azure.perception1.gpu.catkin.clean

        # catkin build the UGV workspaces
        ./deployer -r azure.perception1.gpu.catkin.build

- Please change the robot name `perception1` to whichever Azure robot VM you are building on.

### Automated Catkin Build, Azure CPU VM

Follow this step, **on the localhost**, not on the Azure remote VM. These steps will create the docker image on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the docker shell on the remote host
        ./deployer -r azure.perception1.cpu.docker.shell

        # clean the previous built workspaces
        ./deployer -r azure.perception1.cpu.catkin.clean

        # catkin build the UGV workspaces
        ./deployer -r azure.perception1.cpu.catkin.build

- Please change the robot name `perception1` to whichever Azure robot VM you are building on.

### Manual Catkin Build

#### 1. Access Docker Container (optional)

**If you are not using docker containers, you may skip this step.**

**If you are using an Azure VM, remember to ssh into the VM first.**

        # enter the docker shell container on your local laptop host or Azure VM host#
        #   -- its okay to ignore the error if you have not yet built the workspace: error is: 'bash: /home/developer/deploy_ws/devel/...: No such file or directory'
        docker-join.bash --name perception-gpu-shell

Verify you have gpu passthrough in the docker container:

        nvidia-smi

#### 2. Build Common

The common catkin workspace sets up default `cmake` options.

        # == Common Catkin Workspace ==

        # go to the `common` catkin workspace inside the docker container
        cd ~/deploy_ws/src/common

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set perception-x86

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

#### 3. Build Perception Catkin Workspace

The perception catkin workspace contains all repositories that are running during `SubT` on the perception.

        # go to the `perception::deps` catkin workspace
        cd ~/deploy_ws/src/perception/deps/catkin

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set perception-x86

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

        # go to the `perception` catkin workspace
        cd ~/deploy_ws/src/perception/

        # list the catkin profiles available
        catkin profile list

        # set the catkin profile
        catkin profile set perception-x86

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
        catkin profile set perception-x86

        # view catkin and cmake configuration
        catkin config

        # build the catkin workspace
        catkin build

        # exit the container
        exit

## Cleanup (optional)

You should remove containers when done with its development.

        # stop the running containers
        docker stop perception-cpu-shell
        docker stop perception-gpu-shell

        # remove the containers
        docker rm perception-cpu-shell
        docker rm perception-gpu-shell

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
        ./deployer -r azure.perception1.transfer.to

If you find the `transfer.to` is too slow or missing files during a transfer, you can find the the `transfer.to` options in the file:

        operations/deploy/scenarios/.perception.env

You can edit the option: `deploy_rsync_opts`

- This option tells the deployer to **exclude** files during the transfer. You may change the files that get excluded.
- **Example change:** adding `--exclude=src/.git`, will reduce the time for the transfer, but you wont see any git changes reflected on the remote.