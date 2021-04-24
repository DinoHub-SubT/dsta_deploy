# Robot UAV Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the uav robots.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **basestation**.

## 1. Docker Images

**Create Docker Images**

Follow these steps, **on the basestation**.

```text
# azure registry login
az acr login --name subtexplore

# -- (ROBTS HAVE INTERNET) -- pull the docker images from the azure docker registry
subt deployer robots.uav.ds1.docker.registry.azure.pull

# -- (ROBTS DO NOT HAVE INTERNET) -- pull the docker images from the basestation docker registry

# pull docker images from azure to the basestation
subt deployer local.uav.ds1.docker.registry.azure.pull
```

**Verify Docker Images**

```text
# ssh into your VM (if not already done so), change the below command to match your VM ssh access
ssh uav1.ds

# View the docker images built on the remote VM
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

Follow these steps, **on the basestation**.

```text
# create all the ugv docker containers on all computers
subt deployer robots.uav.ds1.docker.shell.start
```

**Verify Docker Containers**

```text
# ssh into your VM (if not already done so), change the below command to match your VM ssh access
ssh uav.ds1

# view running docker containers
docker ps
```

Verify you see the following docker containers (in any order):

```text
ds1-core-shell
ds1-perception-shell
```

Return To Localhost

```text
# exit the remote VM
exit
```
