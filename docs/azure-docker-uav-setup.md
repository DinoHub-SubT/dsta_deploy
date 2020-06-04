# Azure UAV Docker Setup

### 1. Downloading Thirdparty Software

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

**Copy the thirdparty software to the remote VM**

        # go to the docker opencl thirdparty software directory
        cd ~/deploy_ws/src/operations/deploy/docker/dockerfiles/

        # copy the opencl packages to the remote VM
        # !! -- Please change `azure.uav1` to the Azure VM you are transfering to
        scp -rv thirdparty-software azure.uav1:/home/subt/deploy_ws/src/operations/deploy/docker/dockerfiles

        # verify you have copied the package
        ssh azure.uav1

        # go to the docker opencl thirdparty software directory
        cd ~/deploy_ws/src/operations/deploy/docker/dockerfiles/thirdparty-software/opencl

        # verify you have downloaded the package
        ls -all

        # you should see something like this:
        # -rw-r--r-- 1 ... ... 132125861 l_opencl_p_18.1.0.015.tgz
        # -rw-rw-r-- 1 ... ...      2989 silent-install.cfg

### 2. Building Docker Images

Docker installs all the repository dependencies as *docker images*.

- The docker images will be built on the remote Azure VM.

- You will run the `deployer` tool on the localhost which creates the docker images on the remote Azure VM.

**UAV Docker Image**

Follow this step, **on the localhost**, not on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # build all the uav docker images (this can take a long time)
        # this deployer command will create the docker image on the remote Azure VM.
        ./deployer -s azure.uav1.docker.image

**Access The Remote UAV VM**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh azure.uav1

**Cleanup (Required)**

        # Remove any previously created docker containers (optional).
        #   - its okay to ignore error 'Error: No such container' and continue to the next step.
        docker rm -f uav-sim-shell

        # cleanup dangling docker images
        #   - its okay to ignore error ' "docker rmi" requires at least 1 argument. '
        docker rmi -f $(docker images -f "dangling=true" -q)

**Verify Docker Images**

        # View the docker images built on the localhost
        docker images

        # verify you see the following docker images (in any order):
        #     ->
        #        subt/uav:sim
        #        subt/uav:ros

**Return To Localhost**

        # exit the remote VM
        exit

## 3. Creating Docker Containers With Shell Access

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

**UAV Docker Container Shell Access**

Follow this step, **on the localhost**, not on the Azure remote VM. These steps will create the docker container on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the UAV docker container
        ./deployer -s azure.uav1.docker.shell

**Access The Remote UAV VM**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh azure.uav1

**Verify Docker Containers**

        # view running docker containers
        docker ps

        # verify you see the following docker containers (in any order):
        #   -> uav-shell

**Return To Localhost**

        # exit the remote VM
        exit
