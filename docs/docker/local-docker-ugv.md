# Local UGV Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the localhost laptop.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **localhost**.

## 1. Docker Local Network

To enable multi robot docker simulations, you will need to setup a `docker network`

You only need to **setup the docker network ONCE**. So, if you already created the docker network in another readme, please skip this step.

```text
# create the docker network
subt deployer local.docker.network.create

# verify you have created the network
# - you should see a network called 'robots'
docker network ls

# restart the docker daemon
sudo systemctl restart docker.service
```

**(THIS IS JUST FOR REFERENCE) Keep in mind:**

If you want to use Azure or connect directly to the SubT robots, you must remove the docker network.

- The docker network is on the same subnet, so it will conflict with the Azure or SubT Rajent network.

To remove the docker network:

```text
# remove the docker network
subt deployer local.docker.network.rm

# verify you have remove the network
# - you should not see a network called 'robots'
docker network ls

# restart the docker daemon
sudo systemctl restart docker.service
```

## 3. Docker Images

**Create Docker Images**

Pull the ugv docker images from the Azure docker registry:

```text
# azure registry login
az acr login --name subtexplore

# pull all docker images
subt deployer local.ugv.ugv1.docker.registry.azure.pull

# (optional) pull a single docker images
subt deployer local.ugv.ugv1.core.docker.registry.azure.pull

# (optional) stop & remove any previously created docker containers
subt deployer local.ugv.ugv1.docker.shell.stop
subt deployer local.ugv.ugv1.docker.shell.rm
```

**Verify Docker Images**

```text
# View the docker images built on the localhost
docker images
```

Verify you see the following docker images (in any order):

```text
subt/x86.ugv.cpu.core                                 0.2.c40347f
subt/x86.ugv.cpu.perception                           0.2.c40347f
subt/x86.ugv.cpu.superodometry                        0.2.c40347f
subt/x86.ugv.cpu.ros.melodic                          0.2.c40347f
```

## 4. Docker Containers

**Create Docker Containers**

Create the ugv simulation docker container:

```text
# create the core ugv docker container
subt deployer local.ugv.ugv1.core.docker.shell.start

# (optional) create the perception ugv docker container
subt deployer local.ugv.ugv1.perception.docker.shell.start
```

**Verify Docker Containers**

```text
# View running docker containers (on the remote VM)
docker ps
```

Verify you see the following docker containers (in any order):

```text
# cpu
ugv1-shell
```

## 5. Multi-Robot Simulation

If you wish to run multiple robot simulation on your localhost, you will need to create containers for each robots.

Your multi robot options are:

```text
# create the ugv1 container
subt deployer local.ugv.ugv1.core.docker.shell.start

# create the ugv2 container
subt deployer local.ugv.ugv2.core.docker.shell.start

... there are ugv1-ugv4 options. remember to always TAB complete when trying to find the deployer commands.
```

Each container will have a different IP. You should be able to ping each container (from inside the containers).

When building the catkin workspaces, please just use ugv1 command. You do not need to `catkin` build in all containers (the deploy workspace is mounted, so all containers will use the same `code`, `devel`, `build` paths).

## 6. Comments

When starting the docker container with the deployer and you see the message: `Error response from daemon: network with name robots already exists` **that is OK to ignore**.

If you cannot connect to the docker shell, please notify the maintainer.

## Common Errors

Failed to create docker shell. Example:

```text
Creating ugv1-shell ... error

ERROR: for ugv1-shell  Cannot start service ugv1-core: failed to create endpoint ugv1-shell on network robots: network 11b007d91fb83aa34d9aaec8ba252074bef47c696815c5f76ad701038c8191f1 does not exist

ERROR: for ugv1-core  Cannot start service ugv1-core: failed to create endpoint ugv1-shell on network robots: network 11b007d91fb83aa34d9aaec8ba252074bef47c696815c5f76ad701038c8191f1 does not exist
ERROR: Encountered errors while bringing up the project.
```

Please re-create the container

```text
# remove all previous containers
subt deployer local.docker.ops.rm

# remove docker network
subt deployer local.docker.network.rm

# re-create docker network
subt deployer local.docker.network.create

# create the ugv1 container
subt deployer local.ugv.ugv1.core.docker.shell.start
```