# Azure Basestation Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the remote Azure VM.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **localhost**.

## 1. Docker Images

**Create Docker Images**

Follow these steps, **on the localhost**, not on the Azure remote VM. These steps will create the docker image on the Azure remote VM.

```text
# azure registry login
az acr login --name subtexplore

# pull all the docker images from the azure docker registry
subt deployer azure.basestation.docker.registry.azure.pull

# (optional) stop & remove any previously created docker containers
subt deployer azure.basestation.docker.shell.stop
subt deployer azure.basestation.docker.shell.rm
```

**Verify Docker Images**

```text
# ssh into your VM, change the below command to match your VM ssh access
ssh azure.basestation

# View the docker images built (on the remote VM)
docker images
```

Verify you see the following docker images (in any order):

```text
subt/x86.basestation.cpu.core                         0.2.c40347f
subt/x86.basestation.cpu.ros.melodic                  0.2.c40347f
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
subt deployer azure.basestation.docker.shell.start
```

**Verify Docker Containers**

```text
# ssh into your VM, change the below command to match your VM ssh access
# -- you can switch your docker context instead of ssh
ssh azure.basestation

# view running docker containers
docker ps
```

Verify you see the following docker containers (in any order):

```text
basestation-shell
```

Return To Localhost

```text
# exit the remote VM
exit
```
