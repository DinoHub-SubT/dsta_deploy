# Deployer

> Deployer, a deployment automation tool.

**Table Of Contents**

[TOC]

* * *

# Overview

This deploy repo's purpose is to maintain a working version of the subt workspaces & make robot deployment easier.

Local development, basestation & robot computers can *mostly* all run the deploy repo, in [docker](https://docs.docker.com/get-started/) containers.

The advantage of using the deploy repo & running everything in docker is:

- Deployment on multiple robots.
- Development version are maintained, making issues easier and faster to debug.
- Launch configurations for different robots are version control maintained.
- System dependencies are version control maintained in [dockerfiles](https://docs.docker.com/engine/reference/builder/).
- Continuous integration builds & tests integrated on lab cluster severs.

Deployer automates, on multiple robots, the following tasks:

- Bulding docker images
- Creating & running docker containers
- Building & launching workspaces

The deployer script requires a corresponding *robot deploy configuration yaml*.

- A deploy yaml lists a sequence of shell commands for different tasks.
- Deploy yamls are located in `ci/deploy/field/`

## Clone the Deploy Repo

- Please follow these instructions exactly:

        mkdir ~/deploy_ws
        cd ~/deploy_ws/
        git clone git@bitbucket.org:cmusubt/deploy.git src
        cd src
        ./install-deployer.bash --install
        git submodule update --init --recursive

- **Do not put the deploy repo in another path**, it must be `~/deploy_ws`.

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


## Getting Started with Docker

If you have not installed docker locally or on a robot, [do that first](documentation/docker-install.md) before reading further.

[[top]](#markdown-header-deployer)

* * *

# Deploy Repository Structure Overview

The deploy repo maintains the following:

  - `ci/`
    - jenkins ci configurations
    - `ci/deploy` deployer yaml configurations for workspace clone, docker, transfer, build, launch setups.

  - `rosinstalls` & `submodules`
  
    Update Type       | Workspace
    ------------- | -------------
    rosinstall | basestation, planning, state estimation
    submodules | perception

  - `.catkin_tools`
    - catkin profile build argument configurations for different workspaces.
    - see example: `planning/.catkin_tools/profiles/default/config.yaml` as an example.
      
  - `docker/`
  
    - dockerfiles for building docker images.

  - `launch/`
    - `robot_launch_scripting`: top level launch files.
    - `launch/ugv_setup/logging`: rosbag logging configurations.

  - `tmux/`
    - tmux launch setups

### Workspaces

All the repositories are grouped into workspaces. They have their own [caktin profile](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_profile.html), that setup their catkin build configuration options.

- **Basestation:**

    - `basestation/thirdparty/catkin`
    - `basestation/`

- **Planning**

    - `planning/thirdparty/catkin`
    - `planning/`
    - `planning/hardware/catkin`


- **State Estimation**

    - `state_estimation/thirdparty/catkin`
    - `state_estimation/loam/catkin`
    - `state_estimation/`
    - `state_estimation/xsens/catkin`

- **Perception**

    - `perception/thirdparty/catkin`
    - `perception/objdet_deps/catkin`
    - `perception/`
    - `perception/recording/catkin`

- **Launch**
    
    - `launch/`

To see the different launch profiles do, `catkin profile list` in any of the workspaces.

The `deployer` configuration yamls know which catkin profiles to use for the workspaces.


### Rosinstall To Workspace Relation

- Navigate to the rosinstalls:

        cd /home/$USER/deploy_ws/src/rosinstalls

- **Basestation**:

    rosinstall       | Workspace
    ------------- | -------------
    `basestation/basestation.rosinstall` | `basestation/` 
    `basestation/thirdparty.rosinstall` | `basestation/thirdparty/catkin`

- **Planning**:

    rosinstall       | Workspace
    ------------- | -------------
    `planning/hardware.rosinstall` | `planning/hardware/catkin`
    `planning/thirdparty.rosinstall` | `planning/thirdparty/catkin`
    `planning/planning.rosinstall` | `planning/`

- **State Estimation**:

    rosinstall       | Workspace
    ------------- | -------------
    `state_estimation/loam.rosinstall` | `state_estimation/loam/catkin`
    `state_estimation/state_est.rosinstall` | `state_estimation/`
    `state_estimation/thirdparty.rosinstall` | `state_estimation/thirdparty/catkin`
    `state_estimation/xsens.rosinstall` | `state_estimation/xsens/catkin`

- **Perception**:

    rosinstall       | Workspace
    ------------- | -------------
    `perception/objdet_deps.rosinstall` | `planning/objdet_deps/catkin`
    `perception/recording.rosinstall` | `perception/recording/catkin`
    `perception/thirdparty.rosinstall` | `perception/thirdparty/catkin`

### Submodules

The perception `object_detection` is maintained as a submodule. It does not have a corresponding rosinstall.

[[top]](#markdown-header-deployer)

* * *

# Deployer Script

The deployer script is a python script that automates cloning, building & launching configurations.

The deployer script is run in the following way:

        cd ~/deploy_ws/src
        ./deployer -s [robot].[computer].[action-type].[start/stop]

**The deployer options:**

  - section: `-s` for specifing the yaml section name to execute
  - preview `-p` previewing whether the section execution exists. Does not execute the commands.
  - verbose `-v` shows the commands to execute for the specified sections

**Example:**
  
      ./deployer -s [robot].[computer].[action-type].[start/stop] -p -v

  - make sure the `-p` and `-v` preceed the `-s` section arguments.

The deployer script **must be run from** `cd ~/deploy_ws/src`.

**More Details:**

- See [here](documentation/about-deployer.md) details about the deployer script & its configuration yamls.

[[top]](#markdown-header-deployer)

* * *

# Choosing the Deployment

- There are deployments available:

    Workspaces       | Computer | Robot
    ------------- | -------------| -------------
    basestation | basestation, local workstation | None
    planning | basestation, local workstation, planning-pc, nuc | r1, r2
    state estimation | basestation, local workstation, nuc | r1, r2
    perception | basestation, local workstation, nuc, xavier | r1, r2

## Local Development

- [Local Development](documentation/local/local-basestation.md)
    - local workstation for debugging & development
    - will install `all` workspaces locally

## Robot Basestation

- [Robot Basestation](documentation/local/robot-basestation.md)
    - actual robot basestation used in testing
    - will install `basestation` workspace locally

## Robot Deployment

- [New Robot Deployment](documentation/remote/new-robot-deployment.md)
  
    - will install only `planning, state estimation, perception` workspaces remotely

- [Updating Robot Deployment](documentation/remote/update-robot-deployment.md)
    - will install only `planning, state estimation, perception` workspaces remotely

Once one of the above instructions are completed, **you are done the deployment**.

[[top]](#markdown-header-deployer)

* * *

# Workspace Launch

Everything launches inside the docker container. Meaning you cannot source the workspaces outside of docker.

To enter any docker container:

  - `docker-join` will enter without sourcing any workspaces.
  - `docker-join-[workspace]` will enter with sourcing the workspaces.

Executing `rostopic list` or other ros commands will not work *outside the docker container*. Remember to be inside the container when running ros commands.

## Rosbag Recording

All rosbag recording configurations are found in `launch/ugv_setup/logging/[computer]/script/[computer].bash` in the scripts path.

- For example, see: `launch/ugv_setup/logging/nuc/scripts/nuc.bash`

All rosbags are then found in `launch/ugv_setup/logging/[computer]/bags/` on the robot computer.
    
- There is a soft link in place, to link the `ugv_setup/logging` to `~/logging` path on the robot.

[[top]](#markdown-header-deployer)

* * *

# Issues

- `wstool update` not updating on basestation:
  
    - Make sure the docker container has the network configs setup correctly:
    
            # enter the docker container on the basestation
            docker-join

            # open network configuration files
            sudo vim vim /etc/resolv.conf

            # make sure to add the following:
            nameserver 8.8.8.8

- Docker containers fails to start:

    - If there are any issues starting docker containers:
    
            # removes the docker container
            ./deployer -s [robot].[computer].docker.remove

            # starts the docker container
            ./deployer -s [robot].[computer].docker.start

[[top]](#markdown-header-deployer)


* * *

# Deployment Git Best Practices

When you have updated the deploy repo and are ready to tag a stable commit.

The deploy repo **should pass continuous integration builds** before creating a stable tag. For an introduction to *Jenkins*, please read the [jenkins wiki](https://bitbucket.org/cmusubt/ci_jenkins/wiki/Home).

- **Deploy Branch Conventions:**

    Branch       | Description
    ------------- | -------------
    `master` | Develop branch merged. Always add a stable tag for the PR merge. Should not break jenkins.
    `develop` | Feature/Hotfix branches merged. Not yet always ready for a stable tag. Can break jenkins.
    `[feature/hotfix]/[branch-name]` | Feature branches for development & testing. Can break jenkins.

[[top]](#markdown-header-deployer)
