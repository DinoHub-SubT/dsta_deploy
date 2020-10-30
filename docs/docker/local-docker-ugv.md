# Local UGV Docker Setup

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

        # pull all the docker images from the azure docker registry
        subt deployer local.ugv.ugv1.docker.registry.pull

        # (optional) stop & remove any previously created docker containers
        subt deployer local.ugv.ugv1.docker.stop
        subt deployer local.ugv.ugv1.docker.rm

**Verify Docker Images**

        # View the docker images built (on the remote VM)
        docker images

Verify you see the following docker images (in any order):

        subt/ugv:ppc
        subt/ugv:nuc
        subt/ugv:sim
        subt/ugv:ros

## 2. Creating Docker Containers

**Create Docker Containers**

Follow these steps, **on the localhost**:

        subt deployer local.ugv.ugv1.docker.shell.sim

**Verify Docker Containers**

        # View running docker containers (on the remote VM)
        docker ps

Verify you see the following docker containers (in any order):

        ugv1-sim-shell

## 3. Multi-Robot Simulation

If you wish to run multiple robot simulation on your localhost, you will need to create containers for each robots.

Your multi robot options are:

        # create the ugv1 container
        subt deployer local.ugv.ugv1.docker.shell.sim

        # create the ugv2 container
        subt deployer local.ugv.ugv2.docker.shell.sim

        # create the ugv3 container
        subt deployer local.ugv.ugv3.docker.shell.sim

Each container will have a different IP. You should be able to ping each container (from inside the containers).

When building the catkin workspaces, please just use one of the containers. You do not need to `catkin` build in all containers (the deploy workspace is mounted, so all containers will use the same `code`, `devel`, `build` paths).

## 4. Comments

When starting the docker container with the deployer and you see the message: `Error response from daemon: network with name robots already exists` **that is OK to ignore**.

If you cannot connect to the docker shell, please notify the maintainer.
