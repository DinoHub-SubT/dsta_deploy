# Azure Basestation Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the remote Azure VM.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **localhost**.

## 1. Docker Images

**Create Docker Images**

Follow these steps, **on the localhost**, not on the Azure remote VM. These steps will create the docker image on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # login to azure docker registry
        az acr login --name subtexplore

        # pull all the docker images from the azure docker registry
        ./deployer -s azure.basestation.registry.pull

**Verify Docker Images**

        # ssh into your VM, change the below command to match your VM ssh access
        # -- you can switch your docker context instead of ssh
        ssh azure.basestation

        # View the docker images built (on the remote VM)
        docker images

Verify you see the following docker images (in any order):

        subt/basestation-cpu:0.1
        subt/basestation-cpu:ros

Return To Localhost

        # exit the remote VM
        exit

## 2. Docker Containers

**Create Docker Containers**

Follow these steps, **on the localhost**, not on the Azure remote VM. These steps will create the docker container on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # (optional) remove any previously created docker containers
        ./deployer -s azure.basestation.docker.rm

        # create the basestation docker container
        ./deployer -s azure.basestation.cpu.docker.shell

**Verify Docker Containers**

        # ssh into your VM, change the below command to match your VM ssh access
        # -- you can switch your docker context instead of ssh
        ssh azure.basestation

        # view running docker containers
        docker ps

Verify you see the following docker containers (in any order):

        basestation-cpu-shell

Return To Localhost

        # exit the remote VM
        exit