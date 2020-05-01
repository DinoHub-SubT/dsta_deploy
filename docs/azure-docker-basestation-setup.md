# Azure Basestation Docker Setup

### 1. Building Docker Images

Docker images install all the repository dependencies as *docker images*. The docker images will be built on the remote Azure VM.

**Basestation Docker Image** 

Follow this step, **on the localhost**, not on the Azure remote VM. These steps will create the docker image on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # build the basestation docker image
        ./deployer -s azure.basestation.docker.image

**Access The Remote Basestation VM** 

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh azure.basestation

**Cleanup (Required)**

        # Remove any previously created docker containers (optional).
        #   - its okay to ignore error 'Error: No such container' and continue to the next step.
        docker rm -f gui-shell

        # cleanup dangling docker images
        #   - its okay to ignore error ' "docker rmi" requires at least 1 argument. '
        docker rmi -f $(docker images -f "dangling=true" -q)

**Verify Docker Images** 

        # View the docker images built (on the remote VM)
        docker images

        # verify you see the following docker images (in any order):
        #     ->
        #        subt/basestation:gui
        #        subt/basestation:ros

**Return To Localhost** 

        # exit the remote VM
        exit

## 2. Creating Docker Containers With Shell Access

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

**Basestation Docker Container Shell Access**

Follow this step, **on the localhost**, not on the Azure remote VM. These steps will create the docker container on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the basestation docker container
        ./deployer -s azure.basestation.docker.shell

**Access The Remote Basestation VM** 

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh azure.basestation

**Verify Docker Containers**

        # View running docker containers (on the remote VM)
        docker ps

        # verify you see the following docker containers (in any order):
        #   -> gui-shell

**Return To Localhost** 

        # exit the remote VM
        exit
