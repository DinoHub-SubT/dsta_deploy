# Robot Basestation Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the basestation laptop.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **basestation**.

## 1. Docker Images

**Create Docker Images**

Follow these steps, **on the basestation**.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # pull all the docker images from the azure docker registry
        ./deployer -s azure.basestation.docker.registry.pull

        # push all the azure docker images to the localhost docker registry
        ./deployer -s local.basestation.docker.registry.push

        # remove any previously created docker containers
        ./deployer -s local.basestation.docker.rm

**Verify Docker Images**

        # View the docker images built on the remote VM
        docker images

Verify you see the following docker images (in any order):

        subt/basestation-cpu:0.1
        subt/basestation-cpu:ros

## 2. Docker Containers

**Create Docker Containers**

Follow these steps, **on the basestation**.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create all the basestation docker containers
        ./deployer -s local.basestation.docker.shell

**Verify Docker Containers**

        # view running docker containers
        docker ps

Verify you see the following docker containers (in any order):

        basestation-cpu-shell
