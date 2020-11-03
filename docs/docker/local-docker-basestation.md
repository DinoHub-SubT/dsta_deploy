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

**Keep in mind:**

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
        subt deployer local.basestation.docker.registry.pull

        # (optional) stop & remove any previously created docker containers
        subt deployer local.basestation.docker.stop
        subt deployer local.basestation.docker.rm

**Verify Docker Images**

        # View the docker images built (on the remote VM)
        docker images

Verify you see the following docker images (in any order):

        subt/basestation-cpu:0.1
        subt/basestation-cpu:ros

## 3. Creating Docker Containers

**Create Docker Containers**

Create the basestation docker container:

        subt deployer local.basestation.docker.shell

**Verify Docker Containers**

        # view running docker containers
        docker ps

Verify you see the following docker containers (in any order):

        basestation-cpu-shell

## 4. Comments

When starting the docker container with the deployer and you see the message: `Error response from daemon: network with name robots already exists` **that is OK to ignore**.

If you cannot connect to the docker shell, please notify the maintainer.