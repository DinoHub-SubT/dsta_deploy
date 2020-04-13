# Azure Deployment Example Walkthrough

This tutorial will outline how to setup every workspace on the Azure VM.

- Everything will build and run on the Azure VMs.

- Please only follow the instructions that match your chosen setup.

You can install the workspaces directly on the VM or using docker.

  - The below instructions *(recommended method)* will follow the docker setup.

  - For installing directly on the host VM, please change the docker setup instructions below with the setup found at: `docs/build-local.md`.

## Prerequisites

Assumes the user has already completed the *Azure Cloud Infrastructure Setup* at [`operations/deploy/azurebooks/README.md`](../operations/deploy/azurebooks/README.md) or have a an azure setup available with VM ssh access.

### Setup Configurations

**1. Setup Remote Host Alias**

        # open local host file
        sudo vim /etc/hosts

        # Add all the remote vm hosts. You can configure the names however you like.
        # Example: add the following to the file (please confirm your VM IPs):

        10.0.2.4        azure-ugv
        10.0.2.5        azure-uav
        10.0.2.6        azure-basestation

- You can now ping the remote host using the above alias: `ping azure-ugv`

**2. Setup Remote Host SSH Config**

        # open local ssh config file
        sudo vim ~/.ssh/config

        # Add all the remote vm ssh configuration.  You can configure the names however you like.
        # Example: add the following to the file:

        Host azure.ugv
          HostName azure-ugv
          User [VM USER NAME]   <-- Please enter your VM username here (without brackets)
          IdentitiesOnly yes
          IdentityFile ~/.ssh/azure_vpn

        Host azure.uav
          HostName azure-uav
          User [VM USER NAME]   <-- Please enter your VM username here (without brackets)
          IdentitiesOnly yes
          IdentityFile ~/.ssh/azure_vpn

        Host azure.basestation
          HostName azure-basestation
          User [VM USER NAME]   <-- Please enter your VM username here (without brackets)
          IdentitiesOnly yes
          IdentityFile ~/.ssh/azure_vpn

- You can now ssh to the remote host using the above alias: `ssh azure.ugv`

**3. Setup the Scenario Configuration Files**

The user needs to list IPs for all remote hosts in configuration files called *scenarios files*.

All scenario files can be found in: `operations/deploy/scenarios`

- `azure-[computer]`: configuration for running the deployer on the azure VM
- `desktop-[computer]`: configuration for running the deployer your localhost
- `robot-[computer]`: configuration for running the deployer your the robot deployment

*Personalize the variables*

        # Go to the scenario path
        cd operations/deploy/scenarios

        # edit the appropriate scenario
        gedit operations/deploy/scenarios/[scenario type].env

  - Change `ROS_MASTER_HOSTNAME` to your preference

  - Change `ROS_MASTER_IP` to your preference

  - Change `GPU_ENABLE_TYPE` if your VM has a gpu enabled and nvidia drivers installed.

  - Change `user` to your localhost or remote VM username
  
  - Change `host` to your localhost or remote VM host alias (as edited in /etc/hosts)

  - Change `ssh_config` to your localhost or remote ssh alias (as edited in ~/.ssh/config)

  - Change `LOCAL_DEPLOY_PATH` to the deploy repo path on your localhost

  - Change `REMOTE_DEPLOY_PATH` to the deploy repo path on your remote azure VM (even if it does not exist)

You can change more default variables not listed above or can leave it as the default value.

**4. Transfer SubT Worksapce To Azure VM**

You will want to setup development on the remote VM. There are a few options available:

**Option 1:** Clone the repository directly on the remote host

This option will clone the deploy repository directly on the remote VM and develop as you would on the localhost.

- Follow the top level [`README.md`](../README.md) to clone the deploy workspace directly on the remote Azure VM.

**Option 2:** Mount a remote filesystem using SFTP

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

        # Mount remote directory
        sshfs [desktop username][desktop IP]:/path/to/deploy/workspace/on/locahost /vm1/mountpoint/path/

        # Setup a IDE on localhost with remote editing plugin
        # Example: https://code.visualstudio.com/docs/remote/ssh

        # Remove the remote mount on remote VM host
        sudo umount /vm1/mountpoint/path/

**Option 3:** Deployer Transfer

This option will copy the deploy repo to the remote VM directory.

The transfer needs to be executed anytime you wish to see your local `deploy_ws` changes on the remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # `transfer-to` command does am rsync copy to the Azure VM
        
        # example: transfer to remove uav azure vm
        ./deployer -s azure.uav.transfer.to

        # example: transfer to remove ugv azure vm
        ./deployer -s azure.ugv.transfer.to

        # example: transfer to remove basestation azure vm
        ./deployer -s azure.basestation.transfer.to

* * *

## Quick Start

- Explains how to install workspace dependencies in docker images and build catkin workspaces in docker containers on the remote VM.
- This method is the *recommended method to use* because it is the most straight-forward for repository access and debugging in docker containers.

#### 1. Setup Azure VM Dependencies

This setup (if not already done) only needs to be done once, on a newly created VM.

        # == ssh into your VM ==
        ssh [username]@[private IP]

        # == install deployer dependencies ==

        # install general dependencies
        sudo apt-get update
        sudo apt install -y --no-install-recommends python python-setuptools python-pip
        pip2 install wheel --user
        pip2 install setuptools PyYAML pexpect --user

        # install the deployer tools
        cd ~/deploy_ws/src
        ./install-deployer.bash --install

        # == install docker tools ==

        # install docker
        sudo apt install curl apt-transport-https ca-certificates curl software-properties-common
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add - 
        sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
        sudo apt-get update && sudo apt-get install docker-ce
        sudo usermod -a -G docker $USER

        # install docker compose
        sudo apt-get update
        sudo apt-get install -y --no-install-recommends curl
        sudo curl -L "https://github.com/docker/compose/releases/download/1.24.1/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
        sudo chmod +x /usr/local/bin/docker-compose
        sudo ln -s /usr/local/bin/docker-compose /usr/bin/docker-compose

        # install remote

Verify you have all the third-party operations tools installed correctly:

        # ssh into your VM (if not already done so)
        ssh [username]@[private IP]

        # verify docker
        docker --version

        # verify docker-compose
        docker-compose -v

        # source your bashrc (or zshrc)
        source ~/.bashrc

        # verify docker-compose shows the help usage message
        docker-compose-wrapper --help
        
        # verify deployer script shows the help usage message
        cd ~/deploy_ws/src
        ./deployer --help

#### 2. Build The SubT Workspace

Connect to the Azure VM (if not already done so):

        # == ssh into your VM ==
        ssh [VM username]@[private VM IP]

Build the Workspace

- Build the workspace by following the docker build tutorial: [`docs/build-local-docker.md`](build-local-docker.md) instructions.

#### 4. Summary

If you have completed with the build tutorial correctly, you should now have a built workspace on the Azure VM.

Please notify the maintainer if any of the tutorial steps did not succeed.
