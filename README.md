
# Deployer

> Deployer, a deployment automation tool.

**Table Of Contents**

[TOC]

* * *

# Overview

This deploy repo's maintains a working version of all the subt workspaces in order to make local, robot, cloud deployment somewhat easier.


# Quick Setup

## Prerequisites
  
1. Ubuntu 18.04
   - Not tested on other versions.

2. python2 (does not work with python3)

        # installing python 2.7 and pip
        sudo apt update
        sudo apt install python2.7 python-pip

3. ROS Melodic *(optional)*
   - See the official [ROS install instructions](http://wiki.ros.org/melodic/Installation/Ubuntu).


## SSH Keys

**Bitbucket SSH Keys**

- Generate ssh keys for subt bitbucket:

        mkdir -p ~/.ssh/
        cd ~/.ssh/
        ssh-keygen

    - Answer the prompts from `ssh-keygen` as shown below:
        
            Enter file in which to save the key (/home/<USER-NAME>/.ssh/id_rsa): /home/<USER-NAME>/.ssh/bitbucket
            Enter passphrase (empty for no passphrase):

    - **DO NOT ENTER A PASSPHRASE on `ssh-keygen`! LEAVE IT BLANK.**
    - **Docker will not build successfully if you have a passphrase.**
    - Replace `<USER-NAME>` with your actual username

- Add the generated public key to your bitbucket user: see [**STEP 4**](https://confluence.atlassian.com/bitbucket/set-up-an-ssh-key-728138079.html#SetupanSSHkey-Step4.AddthepublickeytoyourBitbucketsettings)

## Deploy Repository

**1. Install deploy dependencies**

        sudo apt-get update
        sudo apt install -y --no-install-recommends python python-setuptools python-pip
        pip2 install wheel --user
        pip2 install setuptools PyYAML pexpect --user

**2. Clone the deploy repo *(please follow these instructions exactly)* :**

        mkdir ~/deploy_ws/
        cd ~/deploy_ws/
        git clone git@bitbucket.org:cmusubt/deploy.git src
        cd src
        git checkout feature/launch-restructure
        ./install-deployer.bash --install


Notify the maintainer if cloning or installing the deploy repository failed.

## Container Provisioning Tools

### Docker

1. Remove old versions of Docker

        sudo apt-get remove docker docker-engine docker.io

2. Install dependencies and keys

        sudo apt install curl apt-transport-https ca-certificates curl software-properties-common

3. Add the official GPG key of Docker

        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add - 
        sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"


4. Install Docker

        sudo apt-get update && sudo apt-get install docker-ce


5. Add your user to the docker group:

        sudo usermod -a -G docker $USER

    - logout-log-back-in for the changes to take effect


6. Verify your Docker installation

        # verify docker is installed (without sudo access)
        docker -v

    - Do not run with `sudo docker`. Go back to Step 5 if you still cannot run as a non-root user.
    
7. Try running a sample container
    
        sudo docker run hello-world

      - You should see the message *Hello from Docker!* confirming that your installation was successfully completed.

### Docker Compose

1. Download current stable release of *docker compose*
   
        sudo apt-get update
        sudo apt-get install -y --no-install-recommends curl
        sudo curl -L "https://github.com/docker/compose/releases/download/1.24.1/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose

2. Apply executable permissions to the binary

        sudo chmod +x /usr/local/bin/docker-compose
        sudo ln -s /usr/local/bin/docker-compose /usr/bin/docker-compose

3. Test docker compose install
        
        docker-compose --version


### NVIDIA Docker

**Proceed with the below instructions ONLY if you have a NVidia GPU.**

The instructions below assumes you already [installed](https://askubuntu.com/a/1056128) an nvidia driver.

1. Remove old version of Nvidia Docker

        docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f

2. Install NVIDIA Docker

        sudo apt-get purge -y nvidia-docker

3. Setup the NVIDIA Docker Repository

        curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
        distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
        curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
        sudo apt-get update

4. Install NVIDIA Docker (version 2):

        sudo apt-get install -y nvidia-docker2

5. Restart the Docker daemon

        sudo service docker restart

6. Verify your Docker installation

        # verify docker is installed (without sudo access)
        nvidia-docker -v

7. Try running a sample container

        docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi

    - The docker image `nvidia/cuda` requires a recent CUDA version. If you have an earlier CUDA version, then [find a tag](https://hub.docker.com/r/nvidia/cuda/tags) with an earlier version.
        - Example: `docker run --runtime=nvidia --rm nvidia/cuda:8.0-runtime nvidia-smi`
        
    - This command should print your GPU information.


#### Enable NVidia Docker

1. Test NVIDIA Docker runntime is enabled:

        docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi


2. If NVIDIA docker fails to run, with the following error message: `Unknown runtime specified nvidia`


    - Systemd drop-in file

            sudo mkdir -p /etc/systemd/system/docker.service.d
            sudo tee /etc/systemd/system/docker.service.d/override.conf <<EOF
            [Service]
            ExecStart=
            ExecStart=/usr/bin/dockerd --host=fd:// --add-runtime=nvidia=/usr/bin/nvidia-container-runtime
            EOF
            sudo systemctl daemon-reload
            sudo systemctl restart docker

        
    - Daemon configuration file

            sudo tee /etc/docker/daemon.json <<EOF
            {
            "runtimes": {
                    "nvidia": {
                    "path": "/usr/bin/nvidia-container-runtime",
                    "runtimeArgs": []
                    }
            }
            }
            EOF
            sudo pkill -SIGHUP dockerd

    - Try NVIDIA runntime argument again:
        
            docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi        

## Cloud Provisioning Tools

Install the following third-party cloud provisioning operational tools.

### Azure CLI

        # Dependencies
        sudo apt-get update
        sudo apt-get install ca-certificates curl apt-transport-https lsb-release gnupg

        # Software Signing Key
        curl -sL https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/microsoft.asc.gpg > /dev/null

        # Add Azure CLI Software Repository
        AZ_REPO=$(lsb_release -cs)
        echo "deb [arch=amd64] https://packages.microsoft.com/repos/azure-cli/ $AZ_REPO main" | sudo tee /etc/apt/sources.list.d/azure-cli.list

        # Azure CLI Package
        sudo apt-get update
        sudo apt-get install azure-cli

### Terraform

        # Dependencies
        sudo apt-get install unzip wget
        
        # Terraform CLI package
        cd ~/Downloads/
        wget https://releases.hashicorp.com/terraform/0.12.24/terraform_0.12.24_linux_amd64.zip
        unzip terraform_0.12.24_linux_amd64.zip
        sudo mv terraform /usr/local/bin/
        rm terraform_0.12.24_linux_amd64.zip

### Ansible

        sudo apt update
        sudo apt install software-properties-common
        sudo apt-add-repository --yes --update ppa:ansible/ansible
        sudo apt install ansible

### TeamViewer

        # download the teamviewer deb package
        cd /tmp/
        wget https://download.teamviewer.com/download/linux/teamviewer_amd64.deb

        # install teamviewer
        # At the prompt Do you want to continue? [Y/n], type Y to continue the installation.
        sudo apt install ./teamviewer_amd64.deb

        # remove the deb package
        rm teamviewer_amd64.deb

- For more reference, please see [here](https://linuxize.com/post/how-to-install-teamviewer-on-ubuntu-18-04/).

* * *

## Verify Operation Tools Installation

Verify you have all the third-party operations tools installed correctly:

        # verify docker
        docker --version

        # verify docker-compose
        docker-compose -v

        # verify docker-compose
        docker-machine -v

        # verify nvidia-docker
        nvidia-docker -v

        # verify ansible configuration management tools
        ansible --version

        # verify terraform cloud provisioning tool
        terraform --version

        # verify azure cli
        az --help

Verify you can run the deployer scripts installed correctly:

        # source your bashrc (or zshrc)
        source ~/.bashrc

        # verify docker-compose shows the help usage message
        docker-compose-wrapper --help
        
        # verify deployer script shows the help usage message
        ./deployer --help


Notify the maintainer if any of the help usage messages do not show up.

* * *

# Getting Started

The below instructions should get you started on a basic SubT setup locally or on Azure.

## About Operation Tools

There are a few operational tools available to use:

`docker`

  - command interface to interact with `dockerfiles` found in `operations/deploy/docker/dockerfiles`
  
`docker-compose`

  - command interface to interact with `docker-compose.yml` files found in `operations/deploy/docker/dockerfiles/`
  
`docker-compose-wrapper`

  - command interface to wrap `docker compose` and with `scenario` configuration files found in `operations/deploy/scenarios`
  
`docker-machine`

  - command interface to interact with the different azure machine docker daemons.

`deployer`

  - command interface to interact with `deployerfiles` found in `operations/deploy/deploybooks/`
  - automates running the tutorial steps, with realtime command output.

`ansible`

  - command interface to interact with `ansible playbooks` found in `operations/deploy/robotbooks/`
  - automates installing dependencies and setting up systems.

## Operational Tool Resources

Please have a basic understanding of the following the operations tools:

- [Git Submodules](https://www.atlassian.com/git/tutorials/git-submodule)
- [Tmux](https://www.hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/)
- [Docker](https://docs.docker.com/get-started/)
- [Docker Compose (optional)](https://docs.docker.com/compose//)
- [Terraform (optional)](https://www.terraform.io/)
- [Ansible (optional)](https://www.ansible.com/)
- [SubT Deployer Tool (TODO) ](./operations/deploy/deploybooks/README.md)


## Tutorials

You will need to go through a few tutorials to have a working system.

### 1. Initial Repository Setup (Required)

**Tutorial at:** [`docs/install-workspaces.md`](docs/install-workspaces.md)

- Installs all the SubT wrokspace repositories.

### 2. Azure Cloud Infrastructure Setup (Optional)

**Tutorial at:** [`docs/azure-setup.md`](docs/azure-setup.md)

- Follow this tutorial only if you are planning on using the Azure Cloud resource.
- Creates an example azure infrastructure VMs, virtual networking, VPN, etc.
- Sets up remote desktop access.
- Create the docker images, containers on the remote VMs.

### 3. Catkin Build Workspaces (Required)

**Tutorials at**:

  - **Basestation:** [`docs/catkin-basestation.md`](docs/catkin-basestation.md)
  - **UGV:** [`docs/catkin-ugv.md`](docs/catkin-ugv.md)
  - **UAV:** [`docs/catkin-uav.md`](docs/catkin-uav.md)

### 4. Launch Simulation Setup (Required)

**Tutorial 'simple launch example' at**:
  
- **Basestation:** [`docs/launch-basestation.md`](docs/launch-basestation.md)
- **UGV:** [`docs/launch-ugv.md`](docs/launch-ugv.md)



## Operation Tools Summary

You should now have a built `SubT` workspace, either on the localhost or on an Azure setup.

You should become more familiar with the operational tools and their purpose of operations:

**Template: Creating Docker Images**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # Create the docker image
        ./deployer -s [deployment host].[(optional) robot].[(optional) computer].docker.image

        # example: create a docker image on ugv1 Azure VM  (assumes ssh connection available)
        ./deployer -s azure.ugv1.docker.image

**Template: Creating Docker Shell Access Containers**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # Create the docker shell container
        ./deployer -s [deployment host].[(optional) robot].[(optional) computer].docker.shell

        # example: create a docker container with shell access, on ugv1 Azure VM (assumes ssh connection available)
        ./deployer -s azure.ugv1.docker.shell

**Template: Building the catkin workspace**

        # (optional) Access the host
        ssh azure.host-name

        # enter the docker shell container
        docker-join.bash --name [container name]

        # go to the workspace path
        cd ~/deploy_ws/src/path/to/workspace

        # view the catkin profiles
        catkin profile list

        # select the catkin profile
        catkin profile set [profile name]

        # build the workspace
        catkin build

**How to interact with the deployer tool**

To learn more about the available `deployer` tool commands, use the `--preview` or `-p` command as shown below:

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # preview ugv options when deploying on the remote Azure VM
        ./deployer -s azure.ugv1 -p

To learn more what the command executes, use the `--verbose` or `-v` option with the `preview` option as shown below:

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # preview and verbosely display all the commands that will be executed on the Azure VM
        ./deployer -s azure.ugv1 -p -v

* * *

# Discussion

## Remote Development Setup Options

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

**Some Helpful Tools For Remote Development**

- `rdp`, `teamviewer`
- `tmux`, `byobu`
- remote desktop extensions on IDE, for [example](https://code.visualstudio.com/docs/remote/remote-overview).
- `docker machine`, `docker swarm`


## Common Questions

- **When to re-build docker images?**

    - When docker image does not exist on the host ( run `docker images` on the localhost or VM to verify)

    - When new repository dependencies are added to dockerfiles (dockerfiles found in: `operations/deploy/docker/dockerfiles`)

- **When to update dockerfiles?**

    - You should make changes to dockerfiles when you want to need new to add dependencies for workspace repositories (example ros packages, linux packages, etc.). See the existing dockerfiles, found in: `operations/deploy/docker/dockerfiles`, for example dependencies.

    - If you install a dependency in the container directly, remember to put it in the dockerfile and rebuild the docker image.

* * *

# Who do I talk to

* Katarina Cujic (kcujic@andrew.cmu.edu)

[[top]](#markdown-header-deployer)
