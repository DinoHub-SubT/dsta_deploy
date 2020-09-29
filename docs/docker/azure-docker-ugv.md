# Azure UGV Docker Setup

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

        # pull all the docker images from the azure docker registry
        subt deployer azure.ugv.ugv1.docker.registry.pull

        # (optional) stop & remove any previously created docker containers
        subt deployer azure.ugv.ugv1.docker.stop
        subt deployer azure.ugv.ugv1.docker.rm

**Verify Docker Images**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh azure.ugv1

        # View the docker images built on the remote VM
        docker images

Verify you see the following docker images (in any order):

        subt/ugv:ppc
        subt/ugv:nuc
        subt/ugv:sim
        subt/ugv:ros

Return To Localhost

        # exit the remote VM
        exit

## 2. Docker Containers

**Create Docker Containers**

Follow these steps, **on the localhost**, not on the Azure remote VM. These steps will create the docker container on the Azure remote VM.

        subt deployer azure.ugv1.docker.shell.sim

**Verify Docker Containers**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh azure.ugv1

        # view running docker containers
        docker ps

Verify you see the following docker containers (in any order):

        ugv-sim-shell

Return To Localhost

        # exit the remote VM
        exit
