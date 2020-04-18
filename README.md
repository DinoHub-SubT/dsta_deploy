
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
   - please see the official [ROS install instructions](http://wiki.ros.org/melodic/Installation/Ubuntu).


## SSH Keys

**Bitbucket SSH Keys**

- Generate ssh keys for subt bitbucket:

        mkdir -p ~/.ssh/
        cd ~/.ssh/
        ssh-keygen

    - Please answer the prompts from `ssh-keygen` as shown below:
        
            Enter file in which to save the key (/home/<USER-NAME>/.ssh/id_rsa): /home/<USER-NAME>/.ssh/bitbucket
            Enter passphrase (empty for no passphrase):

    - **DO NOT ENTER A PASSPHRASE on `ssh-keygen`! LEAVE IT BLANK.**
    - **Docker will not build successfully if you have a passphrase.**
    - Please replace `<USER-NAME>` with your actual username

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
        ./install-deployer.bash --install


Please notify the maintainer if cloning or installing the deploy repository failed.

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

Please install the following third-party cloud provisioning operational tools.

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


Please notify the maintainer if any of the help usage messages do not show up.

* * *

# Getting Started

The below instructions should get you started on a basic SubT setup locally or on Azure.

## Operational Tool Resources

There are a few operational tools resources available.

Please have a basic understanding of the following tools:

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

Tutorial at: [`docs/install-workspaces.md`](docs/install-workspaces.md)

- Installs all the SubT repositories.

### 2. Azure Cloud Infrastructure Setup (Optional)

Tutorial at: [`operations/deploy/azurebooks/README.md`](operations/deploy/azurebooks/README.md)

- Please follow this tutorial only if you are planning on using the Azure Cloud resource.
- Creates an example azure infrastructure with VMs, virtual networking, VPN.
- Setup remote desktop access.

### 3. Build Setup (Required)

Tutorial at: [`docs/build-tutorial.md`](docs/build-tutorial.md)

- Installs all the workspace dependencies locally or as docker images.
- Setup docker on localhost or on Azure VMs.
- Build all the SubT catkin workspaces.

### 4. Launch Simulation Setup (Required)

Tutorial at:
  
  - **Basestation:** [`docs/launch-basestation.md`](docs/launch-basestation.md)
  - **UGV:** [`docs/launch-ugv.md`](docs/launch-ugv.md)

* * *

# Who do I talk to

* Katarina Cujic (kcujic@andrew.cmu.edu)

[[top]](#markdown-header-deployer)
