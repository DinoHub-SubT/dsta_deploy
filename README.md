
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


**3. python2 (some operational tools do not work with python3 yet)**

        # installing python 2.7 and pip
        sudo apt update
        sudo apt install python2.7 python-pip

**4. ROS Melodic (optional)**

   - See the official [ROS install instructions](http://wiki.ros.org/melodic/Installation/Ubuntu).

**5. Perception Cluster Account (optional)**

  - If you plan on running the UAV or Perception software stacks, you will need access to the `SubT` internal cluster resource (*called Perceptron*)
      - Please notify the maintainer for creating user accounts.
  - Perceptron maintains all the `SubT` rosbag datasets (not all the datasets are found on Azure)
  - Perceptron maintains thirdparty software libraries needed for *some* docker images.

**6. Azure Account (optional)**

  - If you plan on running on Azure, you will access to the `SubT` Azure resource.
      - Please notify the maintainer for creating user accounts.

**7. Localhost disk space requirement**

  - The deploy repo can become large in size. Please have at least 30 GB available on your localhost.

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

- Add the ssh bitbucket key to your localhost ssh config file:

        # create (if not created) ssh config file
        touch ~/.ssh/config

        # open the ssh config file
        gedit ~/.ssh/config

        # Add the following to the top of the config file:
        IdentityFile ~/.ssh/bitbucket

        # exit the ssh config file

## Deploy Repository

**1. Install deploy dependencies**

        sudo apt-get update
        sudo apt install -y --no-install-recommends python python-setuptools python-pip
        pip2 install wheel --user
        pip2 install setuptools PyYAML pexpect --user

**2. Clone the deploy repo *(please follow these instructions exactly)* :**

        # clone & install deploy
        mkdir ~/deploy_ws/
        cd ~/deploy_ws/
        git clone git@bitbucket.org:cmusubt/deploy.git src
        cd src
        git checkout develop
        ./install-deployer.bash --install

        # source your bashrc (or zhsrc, whichever shell you use)
        source ~/.bashrc

- Notify the maintainer if cloning or installing the deploy repository failed.

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
- [SubT Deployer Tool (optional) ](./docs/discuss-operation-tools.md)

* * *

# Getting Started: Tutorials

The below instructions should get you started on a basic `SubT` setup locally or on Azure.

You will need to go through a few tutorials to have a working system.

### 1. Install Third-Party Tool (Required)

**Tutorial at:** [`docs/discuss-install-dependencies.md`](docs/discuss-install-dependencies.md)

- Installs thirdparty tools (`docker, ansible, terraform,` etc.) required to setup deploy.

### 2. Clone Workspaces (Required)

**Tutorial at:** [`docs/discuss-install-workspaces.md`](docs/discuss-install-workspaces.md)

- Installs the `SubT` repositories for all `catkin` workspaces.

### 3. Azure Cloud Infrastructure Setup (Optional)

**Tutorial at:** [`docs/azure-setup.md`](docs/azure-setup.md)

- Follow this tutorial only if you are planning on using the Azure Cloud resource.

This tutorial will setup the following:

- Creates an example azure infrastructure VMs, virtual networking, VPN, etc.
- Sets up remote desktop access.
- Create the docker images, containers on the remote VMs.

### 4. Docker Engine Setup (Required)

If you do not know which tutorial to choose, select the *Azure or Localhost Automated Setup*.

-- | Localhost Automated Setup | Azure Automated Setup | Robot Automated Setup |
--- | ---  |--- | --- |
**Basestation**  | [`docs/local-docker-basestation-setup.md`](docs/local-docker-basestation-setup.md) | [`docs/azure-docker-basestation-setup.md`](docs/azure-docker-basestation-setup.md) | |
**UGV** | [`docs/local-docker-ugv-setup.md`](docs/local-docker-ugv-setup.md) | [`docs/azure-docker-ugv-setup.md`](docs/azure-docker-ugv-setup.md)| |
**UAV** | [`docs/local-docker-uav-setup.md`](docs/local-docker-uav-setup.md) | [`docs/azure-docker-uav-setup.md`](docs/azure-docker-uav-setup.md)| |
**Perception** | | [`docs/azure-docker-perception-setup.md`](docs/azure-docker-perception-setup.md)| |

These tutorials will setup the following:

- Assumes you have already setup the docker engine on the localhost.

- Install all docker images on the remote machines
    - *Docker images setup the all the submodules' package dependencies*

- Create all docker containers on the remote machines
    - *A docker container is the environment where you will be building & running the code*

### 5. Catkin Workspaces Setup (Required)

If you do not know which tutorial to choose, select the Azure or Localhost *Automated Setup*.

-- | Azure or Localhost Manual Setup | Localhost Automated Setup | Azure Automated Setup | Robot Automated Setup |
--- | --- | --- |--- |--- |
**Basestation** |  [`docs/manual-catkin-basestation.md`](docs/manual-catkin-basestation.md) | [`docs/local-catkin-basestation.md`](docs/local-catkin-basestation.md) | [`docs/azure-catkin-basestation.md`](docs/azure-catkin-basestation.md)| |
**UGV** | [`docs/manual-catkin-ugv.md`](docs/manual-catkin-ugv.md) | [`docs/local-catkin-ugv.md`](docs/local-catkin-ugv.md) | [`docs/azure-catkin-ugv.md`](docs/azure-catkin-ugv.md)| |
**UAV** | [`docs/manual-catkin-uav.md`](docs/manual-catkin-uav.md) | [`docs/local-catkin-uav.md`](docs/local-catkin-uav.md) | [`docs/azure-catkin-uav.md`](docs/azure-catkin-uav.md)| |
**Perception** | |  | [`docs/azure-catkin-perception.md`](docs/azure-catkin-perception.md)| |

These tutorials will setup the following:

- Build all the catkin workspaces for the different software stacks.
- Pre-defines the explicit catkin extend paths.
- Pre-defines some cmake and catkin flags (such as using the release flags).


### 6. Simulation Launch Setup (Required)

-- | Azure Tmux Launch | Robot Tmux Launch |
--- | --- | --- | --- | --- |
**Basestation** | [`docs/launch-basestation.md`](docs/launch-basestation.md)| |
**UGV** | [`docs/launch-ugv.md`](docs/launch-ugv.md)| |
**UAV** | [`docs/launch-uav.md`](docs/launch-uav.md)| |

These tutorials will setup the following:

- Launch setups for different software stacks.
- Launch setups for different scenarios (like simple localhost simulation, full Azure simulation, robot hardware, etc.).

### 7. Managing Endpoints (Optional)

**Discussion at:** [`docs/discuss-managing-endpoints.md`](docs/discuss-managing-endpoints.md)

- Methods of managing multiple remote docker endpoints.
- Remote development workflow.

### 8. Updating Deploy Repos (Optional)

**Discussion at:** [`docs/discuss-updating-deploy.md`](docs/discuss-updating-deploy.md)

- Discuss how to update a dockerfile
- Discuss how to update submodules in deploy repo.

### 9. Operational Tools (Optional)

**Discussion at:** [`docs/discuss-operation-tools.md`](docs/discuss-operation-tools.md)

- Operational tools used in deploy, their function and their general syntax.
- Helpful thidparty tools

### 10. More Tutorials (Optional)

Some tutorials are not found on the readme, but on the deploy "wiki" page:

- [Submodules: Getting Started Basics](https://bitbucket.org/cmusubt/deploy/wiki/tutorials/submodules)

### 11. Common Questions and Issues (Optional)

**Discussion at:** [`docs/discuss-questions-issues.md`](docs/discuss-questions-issues.md)

- Common questions
- Known issues

* * *

# Who do I talk to

Please notify the maintainer(s) for any issues or questions:

- Katarina Cujic (`kcujic@andrew.cmu.edu`)

[[top]](#markdown-header-deployer)
