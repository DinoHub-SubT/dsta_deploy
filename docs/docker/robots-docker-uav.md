# Robot UAV Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the uav robots.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **basestation**.

## 1. Docker Images

**Create Docker Images**

Follow these steps, **on the basestation**.

        # azure registry login
        az acr login --name subtexplore

        # (ROBTS HAVE INTERNET) pull the docker images from the azure docker registry
        subt deployer robots.ds.ds1.docker.registry.azure.pull

        # (ROBTS DO NOT HAVE INTERNET) pull the docker images from the basestation docker registry

        # step 1. pull docker images from azure to the basestation
        subt deployer local.uav.uav1.cpu.docker.registry.pull

        # step 2. pull the docker images from the basestation to the robots
        subt deployer robots.ds.ds1.docker.registry.basestation.pull

**Verify Docker Images**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh uav.ds1

        # View the docker images built on the remote VM
        docker images

Verify you see the following docker images (in any order):

        subt/uav-cpu:superodometry
        subt/uav-cpu:uav
        subt/uav-cpu:ros
        subt/uav-gpu:uav
        subt/uav-gpu:ros

Return To Localhost

        # exit the remote VM
        exit

## 2. Docker Containers

**Create Docker Containers**

Follow these steps, **on the basestation**.

        # create all the ugv docker containers on all computers
        subt deployer robots.ds.ds1.docker.shell

**Verify Docker Containers**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh uav.ds1

        # view running docker containers
        docker ps

Verify you see the following docker containers (in any order):

        uav-cpu-shell
        uav-super-shell
        uav-perception-shell

Return To Localhost

        # exit the remote VM
        exit
