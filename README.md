
# Overview

> Deployer, a deployment automation tool.

The deploy repo maintains a working version of all the `SubT` workspaces in order to make local, robot and cloud deployment somewhat easier.

**Table Of Contents**

[TOC]

# Prerequisites

## System Requirements

**1. Linux System: Ubuntu 18.04**

   - Not tested on other versions.

**2. Install system packages without errors**

   - If you have errors and need to do `sudo apt-get install --fix-broken`, you must fix your system package install errors.
   - You will not be able to continue with the below readme until you have fixed all the broken packages.


**3. Python2**

  - Some operational tools do not work with python3 yet


**4. ROS Melodic (optional)**

   - See the official [ROS install instructions](http://wiki.ros.org/melodic/Installation/Ubuntu).

**5. Perception Cluster Account (optional)**

  - Please notify the maintainer for creating user accounts.
  - Perceptron maintains all the `SubT` rosbag datasets (not all the datasets are found on Azure)

**6. Azure Account (required)**

  - If you plan on running on Azure, you will access to the `SubT` Azure resource.
      - Please notify the maintainer for creating user accounts.

**7. Localhost disk space requirement**

  - The deploy repo can become large in size. Please have at least 30 GB available on your localhost.

**8. Localhost User ID**

  - Please make sure your localhost user is id `1000`.
    - i.e. Use the first user you created on your localhost.
    - To check, please run `id`
    - There is a current bug in the docker images that requires this restriction. This will be fixed on next iteration.

**9. Use Standard Bash Terminal (optional)**

  - The SubT deployer autocomplete tab commands do not autocomplete on [`terminator`](https://github.com/gnome-terminator/terminator)
  - If you wish to use the tab autocomplete function, then you can use standard `bash` or `zsh` terminal for best experience.
  - You can still use the deployer commands in `terminator`, you just wont be able to autocomplete.

## Operations Resources

Please have a basic understanding of the following the operational tools:

- [Git](https://git-scm.com/about)
- [Git Submodules](https://www.atlassian.com/git/tutorials/git-submodule)
- [ROS Melodic](http://wiki.ros.org/melodic)
- [Catkin](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html)
- [Docker](https://docs.docker.com/get-started/)
- [Docker Compose (optional)](https://docs.docker.com/compose/)
- [Docker Context (optional)](https://docs.docker.com/engine/context/working-with-contexts/)
- [Azure (optional)](https://docs.microsoft.com/en-us/azure/?product=featured)
- [Azure CLI (optional)](https://docs.microsoft.com/en-us/cli/azure/?view=azure-cli-latest)
- [Terraform (optional)](https://www.terraform.io/)
- [Ansible (optional)](https://www.ansible.com/)
- [Tmux](https://www.hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/)


* * *

# Deploy Repository Setup

### 1. Bitbucket SSH Keys

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

- Add the ssh bitbucket key to your localhost ssh config file:

        # create (if not created) ssh config file
        touch ~/.ssh/config

        # open the ssh config file
        gedit ~/.ssh/config

        # Add the following to the top of the config file:
        IdentityFile ~/.ssh/bitbucket

        # exit the ssh config file


### 2. Clone the deploy repo

        mkdir ~/deploy_ws/
        cd ~/deploy_ws/
        git clone git@bitbucket.org:cmusubt/deploy.git src
        cd src
        git checkout develop

### 3. Install ansible (automated installer script)

        # deployer dependencies
        sudo apt-get update
        sudo apt install -y --no-install-recommends python2.7 python-setuptools python-pip
        pip2 install wheel --user
        pip2 install setuptools PyYAML pexpect --user

        # install ansible
        sudo apt update
        sudo apt install software-properties-common
        sudo apt-add-repository --yes --update ppa:ansible/ansible
        sudo apt install ansible

### 4. Personalize dependency install

Open `operations/deploy/robotbooks/vars/user-inputs.yaml`

- If you have an nvidia card, change from `install_nvidia_docker: false` to `install_nvidia_docker: true`.
- If you want to install ROS melodic, change from `install_ros: false` to `install_ros: true`.

Feel free to change any of the other options.

### 5. Install system dependencies

        # run the deployer operations install
        # - you only need to run this command when there are changes to operations
        # - the maintainer will notify everyone when to re-run this command
        cd ~/deploy_ws/src
        ./install-deployer.bash --install

        # source your bashrc (or zsh or whichever shell you are using)
        # - or open up a new terminal
        source ~/.bashrc

        # run the system dependencies install
        # - please enter your LOCALHOST PASSWORD when prompted
        subtani_install.sh localhost install-localhost.yaml -p

- The ansible installer *might fail* for some users, since everyone has a slightly different system.
- If there are any errors, **please notify the maintainer** and see *Common Ansible Install Errors* below to solve them.

### 6. Verify Installations

Verify you have all the operations tools installed correctly:

        # verify docker
        docker --version

        # verify docker-compose
        docker-compose -v

        # verify nvidia-docker
        nvidia-docker -v

        # verify ansible configuration management tools
        ansible --version

        # verify terraform cloud provisioning tool
        terraform --version

        # verify azure cli
        az --help

        # verify azcopy
        azcopy -v

        # (optional) teamviewer client for remote VM desktop access
        teamviewer --help

        # verify docker-compose shows the help usage message
        docker-compose-wrapper --help

        # verify deployer script shows the help usage message
        ./deployer --help

        # verify SubT autocompleter
        subt help

- Notify the maintainer if any of the `help` usage messages do not show up.

### 7. Common Ansible Install Errors (optional):

**Error: apt-get failed**

- Check `apt-get update` shows NO ERRORS. If so, fix your `apt` sources, before re-running the ansible script.

**Error: permission denied**

- *Solution:*

    - Double check the password you entered. Make sure its the same password you use to login to your user.

- *Solution:*

        sudo visudo

        # Allow user to perform sudo on certain commands, where 'katarina' is my username
        YOUR-USER-NAME ALL=NOPASSWD: ALL

    - This is a security risk, **please remember** to remove once done with the ansible script.

**Error: ansible fails on `docker hello world`**

- Docker still needs to use `sudo` (it should work without `sudo`) because the docker group did not correctly reset in the ansible script.

- *Solution:*

        # reset your user group
        Log out and back in (or restart your computer)

        # retry & confirm docker runs without `sudo`
        docker ps

        # re-run the ansible script
        subtani_install.sh ...

**Notify the maintainer if any of the above deploy setup steps failed.**

* * *

# SubT Getting Started

The below instructions should get you started on a basic `SubT` setup locally, on Azure or on Robots.

You will need to go through a few tutorials to have a working system.

### 1. Clone Workspaces (Required)

**Tutorial at:** [`deploy-clone.md`](docs/deploy-clone.md)

- Installs the `SubT` submodule repositories.

- **If you are DARPA members, then you can skip this step if you already have the code.**

### 2. Docker Registry (Required)

You will need to have an Azure account to access the azure docker registry of where we store docker images.

        # az login will prompt a browser window. Enter your user credentials to login.
        az login

        # login to the subt docker registry
        az acr login --name subtexplore

### 3. Azure Cloud Infrastructure Setup (Optional)

**Tutorial at:** [`azure-setup.md`](docs/azure-setup.md)

- Follow this tutorial only if you are planning on using the Azure Cloud resource.

This tutorial will setup the following:

- Creates an example azure infrastructure VMs, virtual networking, VPN, etc.
- Sets up remote desktop access.
- Create the docker images, containers on the remote VMs.

### 4. Docker Engine Setup (Required)

-- | Localhost Automated Setup | Azure Automated Setup | Robot Automated Setup |
--- | ---  |--- | --- |
**Basestation**  | [`local-docker-basestation.md`](docs/docker/local-docker-basestation.md) | [`azure-docker-basestation.md`](docs/docker/azure-docker-basestation.md) | [`robots-docker-basestation.md`](docs/docker/robots-docker-basestation.md) |
**UGV** | [`local-docker-ugv.md`](docs/docker/local-docker-ugv.md) | [`azure-docker-ugv.md`](docs/docker/azure-docker-ugv.md)| [`robots-docker-ugv.md`](docs/docker/robots-docker-ugv.md) |
**UAV** | [`local-docker-uav.md`](docs/docker/local-docker-uav.md) | [`azure-docker-uav.md`](docs/docker/azure-docker-uav.md)| [`robots-docker-uav-setup.md`](docs/docker/robots-docker-uav.md) |
**Perception** | [`local-docker-perception.md`](docs/docker/local-docker-perception.md) | [`azure-docker-perception-setup.md`](docs/docker/azure-docker-perception.md)| |

### 5. Catkin Workspaces Setup (Required)

-- | Localhost Automated Setup | Azure Automated Setup | Robot Automated Setup |
--- | --- |--- |--- |
**Basestation** |  [`local-catkin-basestation.md`](docs/catkin/local-catkin-basestation.md) | [`azure-catkin-basestation.md`](docs/catkin/azure-catkin-basestation.md) | [`robots-catkin-basestation.md`](docs/catkin/robots-catkin-basestation.md)| |
**UGV** | [`local-catkin-ugv.md`](docs/catkin/local-catkin-ugv.md) | [`azure-catkin-ugv.md`](docs/catkin/azure-catkin-ugv.md) | [`robots-catkin-ugv.md`](docs/catkin/robots-catkin-ugv.md)| |
**UAV** | [`local-catkin-uav.md`](docs/catkin/local-catkin-uav.md) | [`azure-catkin-uav.md`](docs/catkin/azure-catkin-uav.md) | [`robots-catkin-uav.md`](docs/catkin/robots-catkin-uav.md)| |
**Perception** | [`local-catkin-perception.md`](docs/catkin/local-catkin-perception.md) | [`azure-catkin-perception.md`](docs/catkin/azure-catkin-perception.md) | [`robots-catkin-perception.md`](docs/catkin/robots-catkin-perception.md) | |

### 6. Simulation Launch Setup (Required)

-- | Localhost Tmux Launch | Azure Tmux Launch  |
--- | --- | --- | --- | --- |
**Basestation** | [`local-launch-basestation.md`](docs/launch/local-launch-basestation.md)| [`azure-launch-basestation.md`](docs/launch/azure-launch-basestation.md)|
**UGV** | [`local-launch-ugv.md`](docs/launch/local-launch-ugv.md) | [`azure-launch-ugv.md`](docs/launch/azure-launch-ugv.md) |
**UAV** | [`local-launch-uav.md`](docs/launch/local-launch-uav.md) | [`azure-launch-uav.md`](docs/launch/azure-launch-uav.md) |

* * *

## More References

### Managing Endpoints

**Discussion at:** [`discuss-managing-endpoints.md`](docs/discuss-managing-endpoints.md)

- Methods of managing multiple remote docker endpoints.
- Remote development workflow.

### Updating Deploy Repos

**Discussion at:** [`discuss-updating-deploy.md`](docs/discuss-updating-deploy.md)

- Discuss how to update a dockerfile
- Discuss how to update submodules in deploy repo.

### Operational Tools

**Discussion at:** [`discuss-operation-tools.md`](docs/discuss-operation-tools.md)

- Operational tools used in deploy, their function and their general syntax.
- Helpful thidparty tools

### More Tutorials

Some tutorials can be found on the deploy "wiki" page:

- [Submodules: Getting Started Basics](https://bitbucket.org/cmusubt/deploy/wiki/tutorials/submodules)

### Common Questions and Issues

**Discussion at:** [`discuss-questions-issues.md`](docs/discuss-questions-issues.md)

- Common questions
- Known issues

* * *

# Who do I talk to

Please notify the maintainer(s) for any issues or questions:

- Katarina Cujic (`kcujic@andrew.cmu.edu`)

[[top]](#markdown-header-deployer)
