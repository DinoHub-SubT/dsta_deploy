# Azure Basestation Docker Setup

**Note:** Right now, there is no docker image available for perception. You cannot follow these steps right now. This will fixed in the soon future.

### 1. Building Docker Images

Docker images install all the repository dependencies as *docker images*. The docker images will be built on the remote Azure VM.

**Perception Docker Image** 

Follow this step, **on the localhost**, not on the Azure remote VM. These steps will create the docker image on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # build the perception docker image
        ./deployer -s azure.perception.docker.image

**Access The Remote Perception VM** 

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh azure.perception

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
        #        subt/perception:gui
        #        subt/perception:ros

**Return To Localhost** 

        # exit the remote VM
        exit

## 2. Creating Docker Containers With Shell Access

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

**Perception Docker Container Shell Access**

Follow this step, **on the localhost**, not on the Azure remote VM. These steps will create the docker container on the Azure remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # create the perception docker container
        ./deployer -s azure.perception.docker.shell

**Access The Remote Perception VM** 

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh azure.perception

**Verify Docker Containers**

        # View running docker containers (on the remote VM)
        docker ps

        # verify you see the following docker containers (in any order):
        #   -> perception-shell

**Return To Localhost** 

        # exit the remote VM
        exit