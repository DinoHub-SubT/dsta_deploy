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

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # pull all the docker images from the azure docker registry
        ./deployer -s azure.ugv.docker.pull

        # (optional) stop & remove any previously created docker containers
        ./deployer -s local.ugv.docker.stop.all
        ./deployer -s local.ugv.docker.rm.all

**Verify Docker Images**

        # View the docker images built (on the remote VM)
        docker images

Verify you see the following docker images (in any order):

        subt/ugv:sim
        subt/ugv:ros

## 2. Creating Docker Containers

**Create Docker Containers**

Follow these steps, **on the localhost**:

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the ugv docker container
        ./deployer -s local.ugv.docker.shell

**Verify Docker Containers**

        # View running docker containers (on the remote VM)
        docker ps

Verify you see the following docker containers (in any order):

        ugv-sim-shell
