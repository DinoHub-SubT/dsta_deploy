# Robot UAV Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the uav robots.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **basestation**.

## 1. Docker Images

**Create Docker Images**

Follow these steps, **on the basestation**.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # pull all the docker images from the azure docker registry
        ./deployer -s azure.uav.docker.registry.pull

        # TODO: try to remove, add to deployer
        # push all the azure docker images to the localhost docker registry
        ./deployer -s local.uav.docker.registry.push

        # push all the localhost docker images to the robot docker
        ./deployer -s robots.ds1.docker.registry.push

        # Remove any previously created docker containers (example, on ds1 robot)
        ./deployer -s robots.ds1.docker.rm

**Verify Docker Images**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh uav.ds1

        # View the docker images built on the remote VM
        docker images

Verify you see the following docker images (in any order):

        subt/uav-cpu:uav
        subt/uav-cpu:ros
        subt/uav-gpu:uav
        subt/uav-gpu:ros

Return To Localhost

        # exit the remote VM
        exit

## 2. Docker Containers

**Create Docker Containers**

Follow these steps, **on the basestation**.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create all the ugv docker containers on all computers
        ./deployer -s robots.ds1.docker.shell

**Verify Docker Containers**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh uav.ds1

        # view running docker containers
        docker ps

Verify you see the following docker containers (in any order):

        uav-cpu-shell
        uav-perception-shell

Return To Localhost

        # exit the remote VM
        exit
