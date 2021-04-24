# Robot Basestation Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the basestation laptop.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **basestation**.

## 1. Docker Images

**Create Docker Images**

Pull the ugv docker images from the Azure docker registry:

```text
# azure registry login
az acr login --name subtexplore

# pull all the docker images from the azure docker registry
subt deployer robots.basestation.docker.registry.azure.pull

# (optional) stop & remove any previously created docker containers
subt deployer robots.basestation.docker.shell.stop
subt deployer robots.basestation.docker.shell.rm
```

**Verify Docker Images**

```text
# View the docker images built on the remote VM
docker images
```

Verify you see the following docker images (in any order):

```text
subt/x86.basestation.cpu.core                         0.2.c40347f
subt/x86.basestation.cpu.ros.melodic                  0.2.c40347f
```

## 2. Docker Containers

**Create Docker Containers**

Create the basestation docker container:

```text
subt deployer robots.basestation.docker.shell.start
```

**Verify Docker Containers**

```text
# view running docker containers
docker ps
```

Verify you see the following docker containers (in any order):

```text
basestation-shell
```