# Azure Perception Docker Setup

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

# create the docker shell on the remote host
azure.perception.perception1.docker.registry.pull

# (optional) stop & remove any previously created docker containers
subt deployer azure.perception.perception1.docker.stop
subt deployer azure.perception.perception1.docker.rm
```

**Verify Docker Images**

```text
# ssh into your VM (if not already done so), change the below command to match your VM ssh access
ssh azure.perception1

# View the docker images built (on the remote VM)
docker images
```

Verify you see the following docker images (in any order):

```text
subt/perception-gpu:0.1
subt/perception-cpu:0.1
subt/perception-gpu:ros
subt/perception-cpu:ros
```

Return To Localhost

```text
# exit the remote VM
exit
```

## 2. Docker Containers

**Create Docker Containers**

Follow this step, **on the localhost**, not on the Azure remote VM. These steps will create the docker container on the Azure remote VM.

```text
# create the gpu perception docker container
azure.perception.perception1.docker.shell
```

**Verify Docker Containers**

```text
# ssh into your VM (if not already done so), change the below command to match your VM ssh access
ssh azure.perception1

# View running docker containers (on the remote VM)
docker ps
```

Verify you see the following docker containers (in any order):

```text
perception-cpu-shell
ros-cpu-shell
perception-gpu-shell
ros-gpu-shell
```

Return To Localhost

```text
# exit the remote VM
exit
```