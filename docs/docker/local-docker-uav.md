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

## 2. Docker Local Network

To enable multi robot docker simulations, you will need to setup a `docker network`

You only need to **setup the docker network ONCE**. So, if you already created the docker network in another readme, please skip this step.

        # create the docker network
        subt deployer local.docker.network.create

        # verify you have created the network
        # - you should see a network called 'robots'
        docker network ls

        # restart the docker daemon
        sudo systemctl restart docker.service

**Keep in mind:**

If you want to use Azure or connect directly to the SubT robots, you must remove the docker network.

- The docker network is on the same subnet, so it will conflict with the Azure or SubT Rajent network.

To remove the docker network:

        # remove the docker network
        subt deployer local.docker.network.rm

        # verify you have remove the network
        # - you should not see a network called 'robots'
        docker network ls

        # restart the docker daemon
        sudo systemctl restart docker.service

## 3. Docker Images

**Create Docker Images**

Pull the ugv docker images from the Azure docker registry:

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

## 4. Docker Containers

**Create Docker Containers**

Create the uav simulation docker container:

        # create the uav docker container
        # - please user either the cpu or gpu shell. NOT BOTH.
        # - if you're computer has an NVIDIA GPU, then use the gpu shell. Otherwise use the cpu shell.

        # cpu shell
        subt deployer local.uav.uav1.cpu.docker.shell.core

**Verify Docker Containers**

        # View running docker containers (on the remote VM)
        docker ps

Verify you see the following docker containers (in any order):

        # cpu
        uav1-cpu-shell

## 5. Multi-Robot Simulation

If you wish to run multiple robot simulation on your localhost, you will need to create containers for each robots.

Your multi robot options are:

        # create the uav1 container
        subt deployer local.uav.uav1.cpu.docker.shell.core

        # create the uav2 container
        subt deployer local.uav.uav2.cpu.docker.shell.core

        # create the uav3 container
        subt deployer local.uav.uav3.cpu.docker.shell.core

        # create the uav4 container
        subt deployer local.uav.uav4.cpu.docker.shell.core

Each container will have a different IP. You should be able to ping each container (from inside the containers).

When building the catkin workspaces, please just use one of the containers. You do not need to `catkin` build in all containers (the deploy workspace is mounted, so all containers will use the same `code`, `devel`, `build` paths).

## 6. Comments

When starting the docker container with the deployer and you see the message: `Error response from daemon: network with name robots already exists` **that is OK to ignore**.

If you cannot connect to the docker shell, please notify the maintainer.
