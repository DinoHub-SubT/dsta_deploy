# Azure Perception Docker Setup

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

        # change the docker context to the perception1 azure docker daemon
        docker context set azure-perception1

        # pull all the docker images from the azure docker registry
        ./deployer -s azure.perception.docker.registry.pull

        # change the docker context back to the default docker daemon
        docker context set default

        # remove any previously created docker containers
        ./deployer -s azure.perception.docker.rm

**Verify Docker Images**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh azure.perception1

        # View the docker images built (on the remote VM)
        docker images

Verify you see the following docker images (in any order):

        subt/perception-gpu:0.1
        subt/perception-cpu:0.1
        subt/perception-gpu:ros
        subt/perception-cpu:ros

Return To Localhost

        # exit the remote VM
        exit

## 2. Docker Containers

**Create Docker Containers**

Follow this step, **on the localhost**, not on the Azure remote VM. These steps will create the docker container on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the gpu perception docker container
        ./deployer -s azure.perception1.docker.shell

**Verify Docker Containers**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh azure.perception1

        # View running docker containers (on the remote VM)
        docker ps

Verify you see the following docker containers (in any order):

        perception-cpu-shell
        ros-cpu-shell
        perception-gpu-shell
        ros-gpu-shell

Return To Localhost

        # exit the remote VM
        exit
