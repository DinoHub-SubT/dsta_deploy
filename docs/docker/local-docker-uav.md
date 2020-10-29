# Local UAV Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the localhost laptop.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **localhost**.

## 1. Prerequisites

If you have Intel Integrated Graphics, please follow the below instructions with the `cpu` tag.
If you have NVIDIA Graphics, please follow the below instructions, **but substitute the `cpu` tag with the `gpu` tag**.

If you have neither, please notify the maintainer. You will need one of those two options to run.

## 2. Docker Images

**Create Docker Images**

Follow these steps, **on the localhost**.

        # azure registry login
        az acr login --name subtexplore

        # pull the docker images
        subt deployer local.uav.uav1.cpu.docker.registry.pull

        # (optional) stop & remove any previously created docker containers
        subt deployer local.uav.uav1.cpu.docker.stop
        subt deployer local.uav.uav1.cpu.docker.rm

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

## 3. Docker Containers

**Create Docker Containers**

Follow these steps, **on the localhost**.

        # create the uav docker container
        # - please user either the cpu or gpu shell. NOT BOTH.
        # - if you're computer has an NVIDIA GPU, then use the gpu shell. Otherwise use the cpu shell.

        # cpu shell
        subt deployer local.uav.uav1.cpu.docker.shell

**Verify Docker Containers**

        # View running docker containers (on the remote VM)
        docker ps

Verify you see the following docker containers (in any order):

        # cpu
        uav1-cpu-shell

## 4. Multi-Robot Simulation

If you wish to run multiple robot simulation on your localhost, you will need to create containers for each robots.

Your multi robot options are:

        # create the uav1 container
        subt deployer local.uav.uav1.cpu.docker.shell

        # create the uav2 container
        subt deployer local.uav.uav2.cpu.docker.shell

        # create the uav3 container
        subt deployer local.uav.uav3.cpu.docker.shell

        # create the uav4 container
        subt deployer local.uav.uav4.cpu.docker.shell

Each container will have a different IP. You should be able to ping each container (from inside the containers).

When building the catkin workspaces, please just use one of the containers. You do not need to `catkin` build in all containers (the deploy workspace is mounted, so all containers will use the same `code`, `devel`, `build` paths).

## 5. Comments

When starting the docker container with the deployer and you see the message: `Error response from daemon: network with name robots already exists` **that is OK to ignore**.

If you cannot connect to the docker shell, please notify the maintainer.
