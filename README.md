
# Deployer

> Deployer, a deployment automation tool.

**Table Of Contents**

[TOC]

* * *

# Overview

This deploy repo's purpose is to maintain a working version of the subt workspaces & make robot deployment easier.

Local development, basestation & robot computers can *mostly* all run the deploy repo, in [docker](https://docs.docker.com/get-started/) containers.

The advantage of using the deploy repo & running everything in docker is:

- Fast deployment on multiple robots.
- Development version are maintained, making issues easier and faster to debug.
- Launch configurations for different robots are version control maintained.
- System dependencies are version control maintained in [dockerfiles](https://docs.docker.com/engine/reference/builder/).
- Continuous integration builds & tests integrated on lab cluster severs.

Deployer automates, on multiple robots, the following tasks:

- bulding docker images
- creating & running docker containers
- building & launching workspaces

The deployer script requires a corresponding *robot deploy configuration yaml*.

- A deploy yaml lists a sequence of shell commands for different tasks.
- Deploy yamls are located in `ci/deploy/field/`


# Quick Setup: Operation Tools Installation

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

1. Install deploy dependencies

        sudo apt-get update
        sudo apt install -y --no-install-recommends python python-setuptools python-pip
        pip2 install wheel --user
        pip2 install setuptools PyYAML pexpect --user

2. Clone the deploy repo *(please follow these instructions exactly)* :

        mkdir ~/deploy_ws/
        cd ~/deploy_ws/
        git clone git@bitbucket.org:cmusubt/deploy_ws.git src
        cd src
        ./install-deployer.bash --install


Please notify the maintainer if cloning the deploy repository failed.

## Container Provisioning Tools

### Docker

1. Remove old versions of Docker

    `sudo apt-get remove docker docker-engine docker.io`

2. Install dependencies and keys

    `sudo apt install curl apt-transport-https ca-certificates curl software-properties-common`

3. Add the official GPG key of Docker

        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add - 
        sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"


4. Install Docker

    `sudo apt-get update && sudo apt-get install docker-ce`


5. Add your user to the docker group:

    `sudo usermod -a -G docker $USER`

    - logout-log-back-in for the changes to take effect


6. Verify your Docker installation

    * **Please** do not run with `sudo docker`. Go back to Step 5 if you still cannot run as a non-root user.


    *To verify if `docker` is installed:*

    `docker -v`

    *Try running a sample container:*

    `sudo docker run hello-world`

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

* **Proceed with the below instructions only if you have a NVidia GPU.**

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

6. Verify the installation:

    *To verify if `nvidia-docker` is installed:*

    `nvidia-docker -v`

    *Try running a sample container:*

    `docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi`

    - The docker image `nvidia/cuda` requires a recent CUDA version. If you have an earlier CUDA version, then [find a tag](https://hub.docker.com/r/nvidia/cuda/tags) with an earlier version.
        - Example: `docker pull nvidia/cuda:8.0-runtime` and then run the `docker run` command with the `nvidia/cuda:8.0-runtime` image.

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

## Configuration Management Tools

### Ansible

        sudo apt update
        sudo apt install software-properties-common
        sudo apt-add-repository --yes --update ppa:ansible/ansible
        sudo apt install ansible

## Cloud Provisioning Tools

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

* * *

# Verify Operation Tools Installation

Verify you have all the third-party operations tools installed correctly

        # verify docker
        docker --version

        # verify nvidia-docker
        nvidia-docker -v

        # verify docker-compose
        docker-compose -v

        # verify ansible configuration management tools
        ansible --version

        # verify terraform cloud provisioning tool
        terraform --version

        # verify azure cli
        az --help

Verify you can run the deployer scripts installed correctly

        # source your bashrc (or zshrc)
        source ~/.bashrc

        # verify docker-compose shows the help usage message
        docker-compose-wrapper --help
        
        # verify deployer script shows the help usage message
        ./deployer --help


Please notify the maintainer if any of the help usage messages do not show up.

* * *

# Getting The Repositories


When the user clones the deploy workspace, the submodules will not be cloned by default.

The user must decide which *group of submodules* to clone. Submodules are organized into groups of algorithm, computer or robot type.

- Example: `common, ugv uav, basestation`, etc. 

## Required submodules

The user must clone these groups of submodules.

**Clone the common submodules**

    git submodule update --recursive --init common

## Optional submodules

The user only needs to clone what they need. Choose which group of submodules to clone as listed below.

**Clone the basestation submodules**

    git submodule update --recursive --init basestation

**Clone the ugv submodules**

    git submodule update --recursive --init ugv

**Clone the uav submodules**

    git submodule update --recursive --init uav


## Removing Submodules

To remove a group of submodules locally *(for example, when the user is done developing on them)*, perform the following:

    # git command structure template:
    #   -> git submodule deinit -f [ group-name ]

    # example, removing all ugv submodules locally
    git submodule deinit -f ugv

* * *

# Getting Started With Azure & Local Deployment

You must go through both tutorials to have a working system. If not using azure, then skip all *azure* steps and only follow the *localhost* steps.

Quick-start of azure resources setup `operations/deploy/azurebooks/README.md` instructions.

Quick-start of operational deployment setup `operations/deploy/deploybooks/README.md` instructions.

* * *

# Who do I talk to

* Katarina Cujic (kcujic@andrew.cmu.edu)

[[top]](#markdown-header-deployer)
