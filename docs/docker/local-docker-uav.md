# Local UAV Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the localhost laptop.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **localhost**.

## 1. Docker Images

**Create Docker Images**

Follow these steps, **on the localhost**.

        # azure registry login
        az acr login --name subtexplore

        # pull the docker images
        subt deployer local.uav.cpu.docker.registry.pull

        # (optional) stop & remove any previously created docker containers
        subt deployer local.uav.cpu.docker.stop
        subt deployer local.uav.cpu.docker.rm

**Verify Docker Images**

        # View the docker images built on the localhost
        docker images

Verify you see the following docker images (in any order):

        # core
        subt/uav-cpu:uav
        subt/uav-cpu:ros

        # perception
        subt/perception-cpu:0.1
        subt/perception-cpu:ros

## 2. Docker Containers

**Create Docker Containers**

Follow these steps, **on the localhost**.

        # create the uav docker container
        # - please user either the cpu or gpu shell. NOT BOTH.
        # - if you're computer has an NVIDIA GPU, then use the gpu shell. Otherwise use the cpu shell.

        # cpu shell
        subt deployer local.uav.cpu.docker.shell

**Verify Docker Containers**

        # View running docker containers (on the remote VM)
        docker ps

Verify you see the following docker containers (in any order):

        # cpu
        uav-cpu-shell
