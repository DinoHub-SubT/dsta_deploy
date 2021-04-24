# Maintaining Endpoints

If you have completed the full tutorial, you will end up with multiple endpoints and endpoint types.

**Endpoints:**

- Localhost
- Robot Computers
- Azure VMs

**Endpoint Types:**

- Git Repos
- Docker Images
- Docker Containers
- Catkin Workspaces
- Remote Desktops

# Maintaining Deploy Repo Endpoints

## Option 1: Maintain deploy directly on the remote host

Clone the deploy repository directly on the remote VM:

- The user can go directly in the vm workspace and develop.
- The user user can use a IDE remote desktop extension to develop.
- The user can use the `ansible` scripts to re-clone the repo & to setup all the package dependencies.

## Option 2: Deployer Transfer To (rsync)

Transfer the local deploy repository to the remote host, using the `deployer -s ...transfer.to` command:

- The `transfer.to` does a `rsync` between the localhost and remote host deploy workspaces.
- The `transfer.to` is a custom command, that automates `rsync` for different workspaces.
- The transfer uses the setup in the `/etc/hosts`, `~/.ssh/config` for remote access. Please have those setup correctly.

**Example, `transfer.to` to a specific remote Azure VMs:**

```text
# example: transfer to remote basestation azure vm
subt deployer azure.basestation.transfer.to

# example: transfer to the remote ugv1 azure vm
subt deployer azure.ugv.ugv1.transfer.to

# example: transfer to remote uav1 azure vm
subt deployer azure.ugv.uav1.transfer.to

# example: transfer to remote perception1 azure vm
subt deployer azure.perception.perception1.transfer.to
```

#### `Transfer.To` Options

If you find the `transfer.to` is too slow or missing files during a transfer, you can find the the `transfer.to` options in any of the setup files:

```text
operations/scenarios/transfer/basestation.env
operations/scenarios/transfer/ugv.env
operations/scenarios/transfer/uav.env
operations/scenarios/transfer/perception.env
```

You can edit the option: `deploy_rsync_opts`

- This option tells the deployer to **exclude** files during the transfer. You may change the files that get excluded.

Example:

- adding `--exclude=src/.git`, will reduce the time for the transfer, but you wont see any git changes reflected on the remote.


## Option 3: Mount a remote filesystem using SFTP

This option will mount the deploy repo found on the localhost to a directory found on the remote VM.

- This needs to be done only once when the VM starts up and the repositories will be kept in sync.

- You will need to develop on the remote repository, not on the localhost deploy repository.

Find your desktop's IP on the Azure VPN:

- Go to Virtual network gateway
- Go to Point-to-site configuration
- See the Allocated IP addresses

```text
# == ssh into your VM ==
ssh [VM username]@[private VM IP]

# Install sshfs
sudo apt-get install sshfs

# Create remote mount point on localhost
mkdir /vm1/mountpoint/path/

# Mount remote directory (desktop IP is found on Azure Portal VPN connections )
sshfs [desktop username][desktop IP]:/path/to/deploy/workspace/on/locahost /vm1/mountpoint/path/

# Setup a IDE on localhost with remote editing plugin
# Example: https://code.visualstudio.com/docs/remote/ssh

# Remove the remote mount on remote VM host
sudo umount /vm1/mountpoint/path/
```

## Recommendation

There is no good option to choose. Remote development will be difficult to manage.

- A possible recommendation is to use Option 1 and if that becomes inconvenient then try out Option 2.

### Advantages & Disadvantages

**Option 1**

This will require the user to manually manage the VMs, docker containers and to use git as a method of repo sharing.

Use this option if you feel comfortable with git and if you are able to easily switch between the VMs.

**Option 2**

This option will have the user develop on the localhost:

- The `transfer` command does a `rsync` between the localhost and remote host deploy repo. Therefore remotes are always in sync with local, uncommitted changes.
- The user just needs to develop in one place, their localhost.
- The user only needs to use git on the localhost since the remotes are synced.

The issue are:

- The user still has to manage remote docker containers for builds and launches.

- A transfer for a small code change can be slow for the development workflow.

    - Try using `--exclude=src/.git` to speed up the transfer.

- The user might forget to do a `transfer` to the remote VM.

- The `transfer` command can be limited by upload speed.

**Option 3**

This method seems to be very slow.

You can try this option out for experimentation.

* * *

## Maintaining Docker Endpoints

**Things to keep in mind**

- All commands are to be performed on the localhost.
- Requires Azure VM access for the below examples.

### Docker Context Tutorial

**1. View available docker context commands**

```text
docker context --help
```

**2. Verify connection to an example Azure VM**

```text
# Verify you have access to the Basestation Azure VM
ping -c 3 azure-basestation
```

**3. View current setup context**

```text
# view all the context available
docker context ls

# You should see something like this (i.e. output from `docker context ls`):
# NAME                DESCRIPTION                               DOCKER ENDPOINT               KUBERNETES ENDPOINT   ORCHESTRATOR
# default *           Current DOCKER_HOST based configuration   unix:///var/run/docker.sock                         swarm
```

**Things to know:**

- The default context is the docker engine found on the localhost.
- When on the default docker context, you will see all the docker images and containers created on your localhost.

**4. Add a new docker context**

```text
# Add basestation Azure VM docker context
docker context create az-basestation --description "Azure basestation VM" --docker "host=ssh://azure.basestation"

# view all the context available
docker context ls

# You should see something like this (i.e. output from `docker context ls`):
# NAME                DESCRIPTION                               DOCKER ENDPOINT               KUBERNETES ENDPOINT   ORCHESTRATOR
# default *           Current DOCKER_HOST based configuration   unix:///var/run/docker.sock                         swarm
# basestation         Azure basestation VM                      ssh://azure.basestation

# view all docker containers in your current context
docker ps -a

# view all docker images in your current context
docker images
```

**5. Select the new docker context**

```text
# view all the context available
docker context ls

# select the new context (based on step 3.)
docker context use az-basestation

# view the newly selected context
docker context ls

# You should see something like this (i.e. output from `docker context ls`):
# NAME                DESCRIPTION                               DOCKER ENDPOINT               KUBERNETES ENDPOINT   ORCHESTRATOR
# default             Current DOCKER_HOST based configuration   unix:///var/run/docker.sock                         swarm
# basestation *       Azure basestation VM                      ssh://azure.basestation

# view all docker containers in the new context
# -- you should see different containers from the previous context
docker ps -a

# view all docker images in the new context
# -- you should see different imagezs from the previous context
docker images
```

**Things to know:**

- You have now switched to use the docker engine on the remote Azure Basestation VM.
- When on this example basestation docker context, you will see all the docker images and containers created on the remote Azure Basestation VM.

**6. Remove the Example Context**

```text
# view all the context available
docker context ls

# You should see something like this (i.e. output from `docker context ls`):
# NAME                DESCRIPTION                               DOCKER ENDPOINT               KUBERNETES ENDPOINT   ORCHESTRATOR
# default             Current DOCKER_HOST based configuration   unix:///var/run/docker.sock                         swarm
# basestation *       Azure basestation VM                      ssh://azure.basestation

# switch back to the default context
docker context use default

# remove the example basestation context
docker context rm basestation

# view all the context available
docker context ls

# You should see something like this (i.e. output from `docker context ls`):
# NAME                DESCRIPTION                               DOCKER ENDPOINT               KUBERNETES ENDPOINT   ORCHESTRATOR
# default *           Current DOCKER_HOST based configuration   unix:///var/run/docker.sock                         swarm
```

### Issues

**Docker Commands Not Responding**

If you leave the `docker context` pointing to a connection that is not accesible, then all docker commands will not respond.

The docker daemon will be waiting indefinitely for the non-reachable connection.

- For example, setting the docker context to an Azure VM, but the VM that is not accessible. The docker commands will be waiting for the Azure VM connection to come up.

Manual changes to docker config files are needed to get out of the error state:

```text
# open docker context config file
gedit /home/$USER/.docker/config.json

# you will output similar to:
{
        "auths": {},
        "HttpHeaders": {
                "User-Agent": "Docker-Client/19.03.8 (linux)"
        },
        "currentContext": "ugv1"
}

# From: edit the `currentContext` line in the config file.
"currentContext": "ugv1"

# To: change to the default context
"currentContext": "default"
```

You should now be able to run docker commands, for example `docker ps`.

### Summary

You should now be able to switch between the different docker endpoints using the docker context tool.

**Things to keep in mind**

- The docker context commands are to be done on the localhost.

- The docker context tool will switch the docker engine so that you do not have to ssh into the remote machine.
