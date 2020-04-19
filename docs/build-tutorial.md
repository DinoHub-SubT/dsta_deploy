# Build Deployment Operation Tools

[TOC]

## About Operation Tools

There are a few operational tools available to use:

`docker`

  - command interface to interact with `dockerfiles` found in `operations/deploy/docker/dockerfiles`
  
`docker-compose`

  - command interface to interact with `docker-compose.yml` files found in `operations/deploy/docker/dockerfiles/`
  
`docker-compose-wrapper`

  - command interface to wrap `docker compose` and with `scenario` configuration files found in `operations/deploy/scenarios`
  
`docker-machine`

  - command interface to interact with the different azure machine docker daemonss

`deployer`

  - command interface to interact with `deployerfiles` found in `operations/deploy/deploybooks/robot`

`ansible`

  - command interface to interact with `ansible playbooks` found in `operations/deploy/robotbooks/`

* * *
## Tutorial Build Example Walkthrough

Setting up the full build will take a bit of time. The setups are all very similar, so to start, choose one setup and walkthrough the process once. Once more familiar with the tools, walkthrough the entire setup.

Please choose the tutorial that matches your requirements.

### Localhost

TODO: Implementation not yet ready. Please use Azure setup for now.

Builds the `SubT` workspace on the localhost, in docker containers. All dependencies are installed in docker images and maintained in dockerfiles.

Tutorials at:

  - **Basestation:** [`docs/build-basestation-docker.md`](build-basestation-docker.md)
  - **UGV:** [`docs/build-ugv-docker.md`](build-ugv-docker.md)
  - **UAV:** [`docs/build-uav-docker.md`](build-uav-docker.md)


### Azure

Builds the `SubT` workspaces on an azure VMs. The `SubT` workspace is built on the VM, in docker containers.

Tutorials at:

  - **1. Prerequisites:** [`docs/build-azure-prereq.md`](azure-prereq.md)

  - **2. VM Preparation:** [`docs/build-azure-prepare.md`](azure-prepare.md)

  - **3. Workpace Build:** [`docs/build-azure.md`](build-azure.md)

* * *

## Summary

You should now have a built `SubT` workspace.

You should become more familiar with operational tools for different types of operations:

**Template: Creating Docker Images**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # Create the docker image
        ./deployer -s [deployment host].[(optional) robot].[(optional) computer].docker.image

**Template: Creating Docker Shell Access Containers**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # Create the docker shell container
        ./deployer -s [deployment host].[(optional) robot].[(optional) computer].docker.shell

**Template: Building the catkin workspace**

        # enter the docker shell container
        docker-join.bash --name [container name]

        # go to the workspace path
        cd ~/deploy_ws/src/path/to/workspace

        # view the catkin profiles
        catkin profile list

        # select the catkin profile
        catkin profile set [profile name]

To learn more about the available options, please use the `--preview` or `-p` command as example:

        # preview ugv options when deploying on the remote Azure VM
        ./deployer -s azure.ugv1 -p

To learn more about the available options actually do, please use the `--verbose` or `-v` option with the `preview` option

        # preview, verbose the ugv options when deploying on the remote Azure VM
        ./deployer -s azure.ugv1 -p -v

### Common Questions

- *When to re-build docker images:*

    - When docker image does not exist on the host ( `docker images` to verify)
    
    - When  changes are made to dockerfiles (dockerfiles found in: `operations/deploy/docker/dockerfiles`)

- *When to update dockerfiles:*

    - You should make changes to dockerfiles when you want to add a new workspace thirdparty dependency.

    - If you install a dependency in the container directly, please remember to put it in the dockerfile and rebuild the docker image.

* * *

## Remote Development

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

- The `transfer` command can be limited by upload range. On the initial VM setup, please use Option 1 (since the deploy repo can be large).

***Option 1:** Clone the repository directly on the remote host* 

This will require the user to manually manage the VMs, docker containers and to use git as a method of repo sharing.
Use this option if you feel comfortable with git and if you are able to easily switch between the VMs.

***Option 2:** Mount a remote filesystem using SFTP*

This method seems to be vary slow. You can try this out for experimentation.

***Option 3:** Deployer Transfer (manual rsync)*

This option will have the user develop on the localhost. The `transfer` command does a `rsync` between the localhost and remote host deploy workspaces. As well, the user only needs to use git on the localhost since the remotes are synched.

After a period of development, the user issues the `transfer` command which executes the `rsync` between the localhost and remote host deploy workspaces.

The issue are:

- The user still has to manage remote docker containers for builds and launches.

- A transfer for a small code change can be slow for the development workflow.

- The user might forget to do a `transfer` to the remote VM (can transfer to a group of VMs, not just individual).

**Some Helpful Tools For Remote Development**

- `tmux`, `byobu`
- remote desktop extensions on IDE, for [example](https://code.visualstudio.com/docs/remote/remote-overview).
- `docker machine`, `docker swarm`
