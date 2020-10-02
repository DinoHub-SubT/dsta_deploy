# Local Basestation Docker Setup

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
        subt deployer local.basestation.docker.registry.pull

        # (optional) stop & remove any previously created docker containers
        subt deployer local.basestation.docker.stop
        subt deployer local.basestation.docker.rm

**Verify Docker Images**

        # View the docker images built (on the remote VM)
        docker images

Verify you see the following docker images (in any order):

        subt/basestation-cpu:0.1
        subt/basestation-cpu:ros

## 2. Creating Docker Containers

**Create Docker Containers**

Follow these steps, **on the localhost**:

        subt deployer local.basestation.docker.shell

**Verify Docker Containers**

        # view running docker containers
        docker ps

Verify you see the following docker containers (in any order):

        basestation-cpu-shell
