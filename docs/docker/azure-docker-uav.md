# Azure UAV Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the remote Azure VM.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **localhost**.

## 0. Enable GPU Images on Remote Hosts

```text
gedit ~/.subt/user_config.bash

# change the type of docker images to use
export USE_ENHANCED_GPU_DOCKER_IMAGES=true
```

- This is a bug in the deployment operations. Please remember to return it back to your configuration when the docker image pull on azure remote vm is completed.

## 1. Docker Images

**Create Docker Images**

Follow these steps, **on the localhost**, not on the Azure remote VM. These steps will create the docker image on the Azure remote VM.

```text
# azure registry login
az acr login --name subtexplore

# pull all the docker images from the azure docker registry
subt deployer azure.uav.uav1.docker.registry.azure.pull

# (optional) stop & remove any previously created docker containers
subt deployer azure.uav.uav1.docker.shell.stop
subt deployer azure.uav.uav1.docker.shell.rm
```

**Verify Docker Images**

```text
# ssh into your VM (if not already done so), change the below command to match your VM ssh access
ssh azure.uav1

# View the docker images built on the localhost
docker images
```

Verify you see the following docker images (in any order):

```text
subt/x86.uav.cpu.core                                 0.2.c40347f
subt/x86.uav.cpu.perception                           0.2.c40347f
subt/x86.uav.cpu.superodometry                        0.2.c40347f
subt/x86.uav.cpu.ros.melodic                          0.2.c40347f
```

Return To Localhost

```text
# exit the remote VM
exit
```

## 2. Docker Containers

**Create Docker Containers**

Follow these steps, **on the localhost**, not on the Azure remote VM. These steps will create the docker container on the Azure remote VM.

```text
subt deployer azure.uav.uav1.core.docker.shell.start
```

**Verify Docker Containers**

```text
# ssh into your VM (if not already done so), change the below command to match your VM ssh access
ssh azure.uav1

# view running docker containers
docker ps
```

Verify you see the following docker containers (in any order):

```text
uav1-shell
```

Return To Localhost

```text
# exit the remote VM
exit
```
