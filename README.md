
# SubT Deployment Overview

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

# SubT Infrastructure

The below instructions should get you started on a basic `SubT` setup locally, on Azure or on Robots.

You will need to go through a few tutorials to have a working system.

## 1. Getting Started

Please see the [Getting Started With The Setup](./docs/getting-started.md) instructions.

## 2.a. Initialize Workspace

**NOTE: If you are DARPA members, you can skip this step (we provide code directly).**

All deployment operations are accessible by using the tool:

        subt [TAB]

  - Press `TAB` to auto-complete the selections available.
  - Press `TAB SPACE` to show the help information for the auto-complete the selections available.

**Option 1:**

You can use the default git submodule commands such as:

        # clone all the submodules to their  `DETACHED HEAD` commit HASH as pushed on origin.
        git submodule update --init --recursive .

        # (optional) remove all the submodules
        # -- if you ever remove operations submodule, you can re-install it with `./install-deployer.bash --install`
        git submodule deinit -f common basestation simulation ugv uav subt_launch perception

**Option 2:**

Clone the all the submodules, to their latest updates:

        # resets all the submodules, to their `DETACHED HEAD` commit HASH as pushed on origin.
        subt git all.reset

**(Optional)** When you want to pull the latest updates when checked out a branch in submodule:

        # pull the submodule's updates, when the submodules are checked-out at a specific branch.
        subt git all.pull

## 2.b.More Reading


Please read [About Workspace Layout](docs/deploy-layout.md) for getting familiar with the workspace layout.


Please read [Git Helpers](docs/deploy-git.md) for getting familiar with the deployer's git helper tools.

## 3. Docker Registry

You will need to have an Azure account to access the azure docker registry of where we store docker images.

        # az login will prompt a browser window. Enter your user credentials to login.
        az login

        # login to the subt docker registry
        az acr login --name subtexplore

### 4. (Optional) Azure Cloud Infrastructure Setup

Follow the [Azure Setup](docs/azure-setup.md) instructions to setup Azure Cloud resources.

This tutorial will setup the following:

- Creates an example azure infrastructure VMs, virtual networking, VPN, etc.
- Sets up remote desktop access.
- Create the docker images, containers on the remote VMs.

### 5. Docker Engine Setup

-- | Localhost | Azure | Robot |
--- | ---  |--- | --- |
**Basestation**  | [local-docker-basestation.md](docs/docker/local-docker-basestation.md) | [azure-docker-basestation.md](docs/docker/azure-docker-basestation.md) | [robots-docker-basestation.md](docs/docker/robots-docker-basestation.md) |
**UGV** | [local-docker-ugv.md](docs/docker/local-docker-ugv.md) | [azure-docker-ugv.md](docs/docker/azure-docker-ugv.md)| [robots-docker-ugv.md](docs/docker/robots-docker-ugv.md) |
**UAV** | [local-docker-uav.md](docs/docker/local-docker-uav.md) | [azure-docker-uav.md](docs/docker/azure-docker-uav.md)| [robots-docker-uav-setup.md](docs/docker/robots-docker-uav.md) |

### 6. Catkin Workspaces Setup

-- | Localhost | Azure | Robot |
--- | --- |--- |--- |
**Basestation** |  [local-catkin-basestation.md](docs/catkin/local-catkin-basestation.md) | [azure-catkin-basestation.md](docs/catkin/azure-catkin-basestation.md) | [robots-catkin-basestation.md](docs/catkin/robots-catkin-basestation.md)| |
**UGV** | [local-catkin-ugv.md](docs/catkin/local-catkin-ugv.md) | [azure-catkin-ugv.md](docs/catkin/azure-catkin-ugv.md) | [robots-catkin-ugv.md](docs/catkin/robots-catkin-ugv.md)| |
**UAV** | [local-catkin-uav.md](docs/catkin/local-catkin-uav.md) | [azure-catkin-uav.md](docs/catkin/azure-catkin-uav.md) | [robots-catkin-uav.md](docs/catkin/robots-catkin-uav.md)| |

### 7. Simulation Launch Setup

-- | Localhost Launch | Azure Launch |
--- | --- | --- | --- | --- |
**Basestation** | [local-launch-basestation.md](docs/launch/local-launch-basestation.md)| [azure-launch-basestation.md](docs/launch/azure-launch-basestation.md)|
**UGV** | [local-launch-ugv.md](docs/launch/local-launch-ugv.md) | [azure-launch-ugv.md](docs/launch/azure-launch-ugv.md) |
**UAV** | [local-launch-uav.md](docs/launch/local-launch-uav.md) | [azure-launch-uav.md](docs/launch/azure-launch-uav.md) |

* * *

## More References

### Tutorials

Please explore:

    subt tutorial [TAB]

You will find an summarized list of deployer commands, that was outlined in the readme.

Use the `subt tutorial [TAB]` as a quick reference to find commonly used deployer completion commands.

### Updating Submodule Levels

For examples on how to commit on all 3-levels, see [discussion of updating submodules](docs/discuss-updating-deploy.md) instructions.

- Discuss how to update a dockerfile
- Discuss how to update submodules in deploy repo.

- For gif examples, please see [Getting Started Basics](https://bitbucket.org/cmusubt/deploy/wiki/tutorials/submodules) instructions.

### More Operational Tools

For various subt handy operational tools available, see [discussion operation tools](docs/discuss-operation-tools.md) instructions.

- Operational tools used in deploy, their function and their general syntax.
- Helpful thidparty tools

### Managing Endpoints

For the various options available for managing endpoints over remote hosts, please see [discussion for managing endpoints](docs/discuss-managing-endpoints.md) instructions.

- Methods of managing multiple remote docker endpoints.
- Remote development workflow.

### Common Questions and Issues

For common questions and issues please see [discussion of questions readme](docs/discuss-questions-issues.md) instructions.

* * *

# Who do I talk to

Please notify the maintainer(s) for any issues or questions:

- Katarina Cujic (`kcujic@andrew.cmu.edu`)

[[top]](#markdown-header-deployer)
