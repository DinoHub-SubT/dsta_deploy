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
        subt deployer local.ugv.docker.registry.pull

        # (optional) stop & remove any previously created docker containers
        subt deployer local.ugv.docker.stop
        subt deployer local.ugv.docker.rm

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

        subt deployer local.ugv.docker.shell.sim

**Verify Docker Containers**

        # View running docker containers (on the remote VM)
        docker ps

Verify you see the following docker containers (in any order):

        ugv-sim-shell
