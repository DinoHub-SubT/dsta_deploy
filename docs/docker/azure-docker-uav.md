# Azure UAV Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the remote Azure VM.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **localhost**.

## 1. Docker Images

**Create Docker Images**

Follow these steps, **on the localhost**, not on the Azure remote VM. These steps will create the docker image on the Azure remote VM.

        # azure registry login
        az acr login --name subtexplore

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # pull all the docker images from the azure docker registry
        ./deployer -s azure.uav1.docker.pull

        # (optional) stop & remove any previously created docker containers
        ./deployer -s azure.uav1.docker.stop.all
        ./deployer -s azure.uav1.docker.rm.all

**Verify Docker Images**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh azure.uav1

        # View the docker images built on the localhost
        docker images

        # verify you see the following docker images (in any order):
        #     ->
        #        subt/uav:sim
        #        subt/uav:ros

Return To Localhost

        # exit the remote VM
        exit

## 2. Docker Containers

**Create Docker Containers**

Follow this step, **on the localhost**, not on the Azure remote VM. These steps will create the docker container on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the UAV docker container
        ./deployer -s azure.uav1.docker.shell

**Verify Docker Containers**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh azure.uav1

        # view running docker containers
        docker ps

Verify you see the following docker containers (in any order):

        uav-shell

Return To Localhost

        # exit the remote VM
        exit
