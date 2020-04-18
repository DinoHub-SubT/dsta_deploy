# Azure Basestation Docker Setup

Setting up the docker images and containers requires using the `deployer` tool. 

The `deployer` tool hides `docker, docker-compose` details from the user and provides pre-configured options that run the remote docker setup.

- Please notify the maintainer for any errors during any of the below tutorial steps.

Assuming you have `ssh` access to the Azure VMs, please follow the instructions below to setup the VM docker images and containers.

### 1. Building Docker Images

Building docker images on a remote Azure VM. You must perform these commands on your localhost.

**Basestation Docker Image** 

Please follow these steps, **while on the localhost**, not on the Azure remote VM.

These steps will create the docker image on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # build the basestation docker image
        ./deployer -s azure.basestation.docker.image

**Access The Remote Basestation VM** 

        # ssh into your VM (if not already done so), please change the below command to match your VM ssh access
        ssh azure.basestation

**Cleanup (Required)**

        # Remove any previously created docker containers (optional).
        #   - its okay to ignore error 'Error: No such container' and continue to the next step.
        docker rm -f gui-shell

        # cleanup dangling docker images
        docker rmi -f $(docker images -f "dangling=true" -q)

**Verify Docker Images** 

        # View the docker images built (on the remote VM)
        docker images

        # verify you see the following docker images (in any order):
        #     ->
        #        subt/basestation:gui
        #        subt/basestation:ros

**Return To Localhost** 

        # exit the remote VM
        exit

## 2. Creating Docker Containers With Shell Access

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

**Basestation Docker Container Shell Access**

Please follow these steps, **while on the localhost**, not on the Azure remote VM.

These steps will create the docker container on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the basestation docker container
        ./deployer -s azure.basestation.docker.shell

**Access The Remote Basestation VM** 

        # ssh into your VM (if not already done so), please change the below command to match your VM ssh access
        ssh azure.basestation

**Verify Docker Containers**

        # View running docker containers (on the remote VM)
        docker ps

        # verify you see the following docker containers (in any order):
        #   -> gui-shell

**Return To Localhost** 

        # exit the remote VM
        exit
