# Local Basestation Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the localhost laptop.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **localhost**.

## 1. Docker Local Network

To enable multi robot docker simulations, you will need to setup a `docker network`

You only need to **setup the docker network ONCE**. So, if you already created the docker network in another readme, please skip this step.

        # create the docker network
        subt deployer local.docker.network.create

        # verify you have created the network
        # - you should see a network called 'robots'
        docker network ls

        # restart the docker daemon
        sudo systemctl restart docker.service

**(THIS IS JUST FOR REFERENCE) Keep in mind:**

If you want to use Azure or connect directly to the SubT robots, you must remove the docker network.

- The docker network is on the same subnet, so it will conflict with the Azure or SubT Rajent network.

To remove the docker network:

        # remove the docker network
        subt deployer local.docker.network.rm

        # verify you have remove the network
        # - you should not see a network called 'robots'
        docker network ls

        # restart the docker daemon
        sudo systemctl restart docker.service

## 2. Docker Images

**Create Docker Images**

Pull the ugv docker images from the Azure docker registry:

        # azure registry login
        az acr login --name subtexplore

        # pull all the docker images from the azure docker registry
        subt deployer local.basestation.docker.registry.azure.pull

        # (optional) stop & remove any previously created docker containers
        subt deployer local.basestation.docker.shell.stop
        subt deployer local.basestation.docker.shell.rm

**Verify Docker Images**

        # View the docker images built (on the remote VM)
        docker images

Verify you see the following docker images (in any order):

        subt/x86.basestation.cpu.core                         249324c
        subt/x86.basestation.cpu.ros.melodic                  249324c

## 3. Creating Docker Containers

**Create Docker Containers**

Create the basestation docker container:

        subt deployer local.basestation.core.docker.shell.start

**Verify Docker Containers**

        # view running docker containers
        docker ps

Verify you see the following docker containers (in any order):

        basestation-shell

## 4. Comments

When starting the docker container with the deployer and you see the message: `Error response from daemon: network with name robots already exists` **that is OK to ignore**.

If you cannot create the docker shell, please notify the maintainer.

## Common Errors

Failed to create docker shell. Example:

        Creating basestation-shell ... error

        ERROR: for basestation-shell  Cannot start service basestation-core: failed to create endpoint basestation-shell on network robots: network 11b007d91fb83aa34d9aaec8ba252074bef47c696815c5f76ad701038c8191f1 does not exist

        ERROR: for basestation-core  Cannot start service basestation-core: failed to create endpoint basestation-shell on network robots: network 11b007d91fb83aa34d9aaec8ba252074bef47c696815c5f76ad701038c8191f1 does not exist
        ERROR: Encountered errors while bringing up the project.

Please re-create the container

        # remove all previous containers
        subt deployer local.docker.ops.rm

        # remove docker network
        subt deployer local.docker.network.rm

        # re-create docker network
        subt deployer local.docker.network.create

        # re-create the basestation docker container:
        subt deployer local.basestation.core.docker.shell.start
