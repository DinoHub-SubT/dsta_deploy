# Build With Azure Walkthrough

This tutorial will outline how to setup every workspace on the Azure VM.

- Everything will build and run on the Azure VMs.

- Please only follow the instructions that match your chosen setup.

You can install the workspaces directly on the VM or using docker.

  - The below instructions *(recommended method)* will follow the docker setup.

Please make sure you have completed the *Build Azure Prerequisites* instructions.

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

- Build the workspace by following any of the docker build tutorial instructions:

    - Basestation: [`docs/build-basestation-docker.md`](build-basestation-docker.md)
    - UGV: [`docs/build-ugv-docker.md`](build-ugv-docker.md)
    - UAV: [`docs/build-uav-docker.md`](build-uav-docker.md)


#### 3. Summary

You should now have a built `SubT` workspace on a Azure VM.

- Please notify the maintainer if any of the tutorial steps did not succeed.

Please go back to [`build-tutorial`](build-tutorial.md#Summary) for summary comments.