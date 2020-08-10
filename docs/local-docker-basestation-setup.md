# Local Basestation Docker Setup

## Local CPU VM

If you do not have a GPU on your localhost, then follow these instructions.

### 1. Building Docker Images

Docker install all the repository dependencies as *docker images*.

- The docker images will be built on the remote localhost.

**Create the Docker Image on LocalHost**

These steps will create the docker image on the localhost.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # build the basestation docker image
        ./deployer -s local.basestation.cpu.docker.image

**Cleanup (Required)**

        # Remove any previously created docker containers (optional).
        #   - its okay to ignore error 'Error: No such container' and continue to the next step.
        docker rm -f basestation-cpu-shell

        # cleanup dangling docker images
        #   - its okay to ignore error ' "docker rmi" requires at least 1 argument. '
        docker rmi -f $(docker images -f "dangling=true" -q)

**Verify Docker Images**

        # View the docker images built (on the remote VM)
        docker images

        # verify you see the following docker images (in any order):
        #     ->
        #        subt/basestation-cpu:0.1
        #        subt/basestation-cpu:ros


### 2. Creating Docker Containers With Shell Access

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

**Create the Docker Shell on LocalHost**

These steps will create the docker container on the localhost.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the basestation docker container
        ./deployer -s local.basestation.cpu.docker.shell

**Verify Docker Containers**

        # View running docker containers (on the remote VM)
        docker ps

        # verify you see the following docker containers (in any order):
        #   -> basestation-cpu-shell

* * *

## Localhost GPU VM

If you do have a GPU on your localhost, then follow these instructions.

### 1. Building Docker Images

Docker install all the repository dependencies as *docker images*.

- The docker images will be built on the localhost.

**Create the Docker Images on Localhost**

These steps will create the docker image on the localhost.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # build the basestation docker images
        ./deployer -s local.basestation.cpu.docker.image
        ./deployer -s local.basestation.gpu.docker.image

**Cleanup (Required)**

        # Remove any previously created docker containers (optional).
        #   - its okay to ignore error 'Error: No such container' and continue to the next step.
        docker rm -f basestation-cpu-shell basestation-gpu-shell

        # cleanup dangling docker images
        #   - its okay to ignore error ' "docker rmi" requires at least 1 argument. '
        docker rmi -f $(docker images -f "dangling=true" -q)

**Verify Docker Images**

        # View the docker images built (on the remote VM)
        docker images

        # verify you see the following docker images (in any order):
        #     ->
        #        subt/basestation-gpu:0.1
        #        subt/basestation-gpu:ros
        #        subt/basestation-cpu:0.1
        #        subt/basestation-cpu:ros

### 2. Creating Docker Containers With Shell Access

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

**Create the Docker Shell on LocalHost**

These steps will create the docker container on the localhost.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the basestation docker container
        ./deployer -s local.basestation.cpu.docker.shell
        ./deployer -s local.basestation.gpu.docker.shell

**Verify Docker Containers**

        # View running docker containers (on the remote VM)
        docker ps

        # verify you see the following docker containers (in any order):
        #   -> basestation-cpu-shell, basestation-gpu-shell
