# Local UAV Docker Setup

## 1. Downloading Thirdparty Software

The UAV docker images requires installing thirdparty software, not found using `apt-get`.

You will need to download the software from the `SubT` Cluster `Perceptron`, to your deploy repo's docker folder on your localhost.

- Please make sure you have a `Perceptron` account. If not, ask the maintainer to create you a user account.

**Download OpenCL Runtime Library**

        # go to the docker opencl thirdparty software directory
        cd ~/deploy_ws/src/operations/deploy/docker/dockerfiles/thirdparty-software/opencl

        # Download OpenCL runtime library from Perceptron
        # !! -- Please change `USERNAME` to your perceptron username
        scp -vr -o IdentitiesOnly=yes USERNAME@perceptron.ri.cmu.edu:///project/subt/data/deploy-operations/thirdparty-software/l_opencl_p_18.1.0.015.tgz .

        # verify you have downloaded the package
        ls -all

        # you should see something like this:
        # -rw-r--r-- 1 ... ... 132125861 l_opencl_p_18.1.0.015.tgz
        # -rw-rw-r-- 1 ... ...      2989 silent-install.cfg

## Local CPU VM

If you do not have a GPU on your localhost, then follow these instructions.

## 1. Building Docker Images

Docker install all the repository dependencies as *docker images*.

- The docker images will be built on the remote localhost.

**Create the Docker Image on LocalHost**

These steps will create the docker image on the localhost.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # build the uav docker image
        ./deployer -s local.uav.cpu.docker.image

**Cleanup (Required)**

        # Remove any previously created docker containers (optional).
        #   - its okay to ignore error 'Error: No such container' and continue to the next step.
        docker rm -f uav-cpu-shell

        # cleanup dangling docker images
        #   - its okay to ignore error ' "docker rmi" requires at least 1 argument. '
        docker rmi -f $(docker images -f "dangling=true" -q)

**Verify Docker Images**

        # View the docker images built (on the remote VM)
        docker images

        # verify you see the following docker images (in any order):
        #     ->
        #        subt/uav-cpu:uav
        #        subt/uav:ros

## 2. Creating Docker Containers With Shell Access

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

**Create the Docker Shell on LocalHost**

These steps will create the docker container on the localhost.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the uav docker container
        ./deployer -s local.uav.cpu.docker.shell

**Verify Docker Containers**

        # View running docker containers (on the remote VM)
        docker ps

        # verify you see the following docker containers (in any order):
        #   -> uav-cpu-shell

## Localhost GPU VM

If you do have a GPU on your localhost, then follow these instructions.

## 1. Building Docker Images

Docker install all the repository dependencies as *docker images*.

- The docker images will be built on the remote localhost.

**Create the Docker Image on LocalHost**

These steps will create the docker image on the localhost.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # build the uav docker image
        ./deployer -s local.uav.gpu.docker.image

**Cleanup (Required)**

        # Remove any previously created docker containers (optional).
        #   - its okay to ignore error 'Error: No such container' and continue to the next step.
        docker rm -f uav-gpu-shell

        # cleanup dangling docker images
        #   - its okay to ignore error ' "docker rmi" requires at least 1 argument. '
        docker rmi -f $(docker images -f "dangling=true" -q)

**Verify Docker Images**

        # View the docker images built (on the remote VM)
        docker images

        # verify you see the following docker images (in any order):
        #     ->
        #        subt/uav-gpu:uav
        #        subt/uav:ros

## 2. Creating Docker Containers With Shell Access

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

**Create the Docker Shell on LocalHost**

These steps will create the docker container on the localhost.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the uav docker container
        ./deployer -s local.uav.gpu.docker.shell

**Verify Docker Containers**

        # View running docker containers (on the remote VM)
        docker ps

        # verify you see the following docker containers (in any order):
        #   -> uav-gpu-shell
