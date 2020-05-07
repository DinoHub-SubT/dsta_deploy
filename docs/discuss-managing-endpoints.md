# About Managing Endpoints

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

# Maintaining Docker Endpoints

You now will have setup multiple docker images and containers on different endpoints.

You can manage docker on the different endpoints using the tool `docker context`.

**Endpoints are defined as:**

- Localhost
- Robot Computers
- Azure VMs

## Docker Context Tutorial

**Things to keep in mind:**

- All commands are to be performed on the localhost.
- Requires Azure VM access for the below examples.

**1. View available docker context commands**

        docker context --help

**2. Verify connection to an example Azure VM**

        # Verify you have access to the Basestation Azure VM
        ping -c 3 azure-basestation

**3. View current setup context**

        # view all the context available
        docker context ls

        # You should see something like this (i.e. output from `docker context ls`):
        # NAME                DESCRIPTION                               DOCKER ENDPOINT               KUBERNETES ENDPOINT   ORCHESTRATOR
        # default *           Current DOCKER_HOST based configuration   unix:///var/run/docker.sock                         swarm

**Things to know:**
- The default context is the docker engine found on the localhost.
- When on the default docker context, you will see all the docker images and containers created on your localhost.

**4. Add a new docker context**

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

**5. Select the new docker context**

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

**Things to know:**
- You have now switched to use the docker engine on the remote Azure Basestation VM.
- When on this example basestation docker context, you will see all the docker images and containers created on the remote Azure Basestation VM.

**6. Remove the Example Context**

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

## Summary

You should now be able to switch between the different docker endpoints using the docker context tool.

**Things to keep in mind:**
- The docker context commands are to be done on the localhost.
- The docker context tool will switch the docker engine so that you do not have to ssh into the remote machine.

**Example Use Cases**

*Access a container on the remote Basestation Azure VM:*

-  Switch to the Azure Basestation docker context.
-  Then perform `docker-join`.
-  You will enter the container found on the remote Basestation Azure VM.

# Remote Development Workflow Discussion

You will want to setup development workflow on the remote VM. There are a few options available:

**Option 1: Clone the repository directly on the remote host**

This option will clone the deploy repository directly on the remote VM and develop as you would on the localhost. Then the user can use a remote desktop extension to develop or go directly in the vm and develop.

- Use the `ansible` scripts to setup all the package dependencies and to clone the deploy repo on new VMs.

**Option 2: Mount a remote filesystem using SFTP**

This option will mount the deploy repo found on the localhost to a directory found on the remote VM.

- This needs to be done only once when the VM starts up and the repositories will be kept in sync.

- You will need to develop on the remote repository, not on the localhost deploy repository.

Find your desktop's IP on the Azure VPN:

- Go to Virtual network gateway
- Go to Point-to-site configuration
- See the Allocated IP addresses

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

**Option 3: Deployer Transfer (manual rsync)**

This option will copy the deploy repo to the remote VM directory.

The transfer command does a `rsync` between the localhost and remote host deploy workspaces.

- The transfer command references the setup in the `/etc/hosts` and in the `~/.ssh/config`. Please have those setup correctly.

An example `transfer` command will have the following template format:

        # go to the deploy top level path
        cd ~/deploy_ws/src
        
        # example: transfer to remote uav1 azure vm
        ./deployer -s azure.uav1.transfer.to

        # example: transfer to the remote ugv1 azure vm
        ./deployer -s azure.ugv1.transfer.to

        # example: transfer to remote basestation azure vm
        ./deployer -s azure.basestation.transfer.to

**Recommendation**

There is no good option to choose. Remote development will be difficult to manage.

- A possible recommendation is to use Option 1 and if that becomes inconvenient then try out Option 3.

- The `transfer` command can be limited by upload range. On the initial VM setup, use Option 1 (since the deploy repo can be large).

**Option 1:** Clone the repository directly on the remote host

This will require the user to manually manage the VMs, docker containers and to use git as a method of repo sharing.
Use this option if you feel comfortable with git and if you are able to easily switch between the VMs.

**Option 2:** Mount a remote filesystem using SFTP

This method seems to be very slow. You can try this out for experimentation.

**Option 3:** Deployer Transfer (manual rsync)

This option will have the user develop on the localhost. The `transfer` command does a `rsync` between the localhost and remote host deploy repo. So, the user only needs to use git on the localhost since the remotes are synced.

The issue are:

- The user still has to manage remote docker containers for builds and launches.

- A transfer for a small code change can be slow for the development workflow.

- The user might forget to do a `transfer` to the remote VM (there is the option available to transfer to a group of VMs, not just individual VM).
