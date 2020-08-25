# Robot UGV Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the ugv robots.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **basestation**.

## 1. Docker Images

**Create Docker Images**

Follow these steps, **on the basestation**.

        # azure registry login
        az acr login --name subtexplore

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # pull all the docker images from the azure docker registry
        ./deployer -s azure.ugv1.ppc.docker.pull
        ./deployer -s azure.ugv1.nuc.docker.pull

        # (optional) stop & remove any previously created docker containers
        ./deployer -s robots.ugv1.docker.stop.all
        ./deployer -s robots.ugv1.docker.rm.all

**Verify Docker Images**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh azure.ugv1

        # View the docker images built on the remote VM
        docker images

Verify you see the following docker images (in any order):

        # only found on the ugv planning-pc
        subt/ugv:ppc
        subt/ugv:nuc
        subt/ugv:ros

        # only found on the ugv nuc
        subt/ugv:nuc
        subt/ugv:ros

Return To Localhost

        # exit the remote VM
        exit

## 2. Docker Containers

**Create Docker Containers**

Follow these steps, **on the basestation**.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create all the ugv docker containers on all computers
        ./deployer -s robots.ugv1.docker.shell

**Verify Docker Containers**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh ugv1.ppc

        # view running docker containers
        docker ps

Verify you see the following docker containers (in any order):

        # only found on the ugv planning-pc
        ppc-shell
        # only found on the ugv nuc
        nuc-shell

Return To Localhost

        # exit the remote VM
        exit
