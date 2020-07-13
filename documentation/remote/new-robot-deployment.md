# New Robot Deployment

**Table Of Contents**

[TOC]

## Overview

During any remote deployment, **always remember** to **transfer the deploy workspace from the basestation to the robot**.

- There are different robot & robot-computer configuration available.

    Workspaces       | Computer | Robot
    ------------- | -------------| -------------
    planning | planning-pc, nuc | r1, r2
    state estimation | nuc | r1, r2
    perception | nuc, xavier | r1, r2


## Install Docker on the Robot

Follow the [docker install](#markdown-header-install-docker) instructions:

  - **Do not** follow the *Install Nvidia Docker*. Only follow the basic docker install instructions.

  - internet access required.

Once installed once, you do not need to install again.

## SSH Keys

Robots require two types of keys for deployment:

1. **Deploy SSH Keys**
   - Create a ssh key between the robot computers, to allow the deployer ssh password-less access. Call these keys `deploy`.

   - Generate ssh keys

           cd ~/.ssh/
           ssh-keygen

       - Please answer the prompts from `ssh-keygen` as shown below:
           
               Enter file in which to save the key (/home/<USER-NAME>/.ssh/id_rsa): /home/<USER-NAME>/.ssh/deploy
               Enter passphrase (empty for no passphrase):

       - **DO NOT ENTER A PASSPHRASE on `ssh-keygen`! LEAVE IT BLANK.**
       - **Deployer will not run successfully if you have a passphrase.**
       - Please replace `<USER-NAME>` with your actual username

   - Add the generated public key to robot's other computers' `~/.ssh/authorized_keys` to allow the deployer to have password-less ssh access.

2. **Bitbucket SSH Keys**

   - Generate ssh keys

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


## Choosing the Deployment

**Follow the instructions:**

- [local deploy](#markdown-header-local-deploy)
    -  if you are deploying *directly on the robot*, not over the basestation.

- [remote deploy](#markdown-header-remote-deploy)
    - if you are deploying *over the basestation*.

Please do not do local deployment on robot, except **only if necessary**. Please see *update-robot-deployment* to do deployment over the basestation instead.

### Local Deploy

#### Clone the deploy workspace

- Clone the deploy repo:

        # make sure to create the deploy_ws in this location        
        mkdir /home/$USER/deploy_ws

        # clone the deploy repo as src
        git clone git@bitbucket.org:cmusubt/deploy.git src
        cd src

- Install the deployer dependencies:

        sudo apt-get update && sudo apt-get install python-pip -y --no-install-recommends
        pip install setuptools PyYAML pexpect --user

- Setup the deploy script:
        
        # deployer script must always be called from the deploy_ws/src path
        cd /home/$USER/deploy_ws/src

        # install the deployer
        ./install-deployer.bash --install

        # source your bash or zsh
        source ~/.bashrc
        source ~/.zshrc

- Verify the deploy is working:

        ./deployer --help

#### Setup the robot workspace

- Navigate to the deploy repo:

        cd /home/$USER/deploy_ws/src

- Build the subt docker images:

        # build the docker images on the basestation
        ./deployer -s [robot].[computer].docker.image --local

    - internet access required.

- Start the docker container:

        ./deployer -s [robot].[computer].docker.start --local

- Clone all the submodules & rosinstall repos:
    
        # initialize all the repos to install
        ./deployer -s [robot].[computer].init --local

        # clone all the repos using wstool and submodule update
        ./deployer -s [robot].[computer].up --local

#### Build the robot workspace

- Start the docker container *(if not already started)*:

        ./deployer -s [robot].[computer].docker.start --local

- Robot-computer specpfic pre-build steps:

    - **nuc:**
    
            # install thirdparty xsens libraries in the container before building
            ./deployer -s [robot].nuc.rosdep.xsens --local

- Build the robot workspace:
    
        # this will build the workspace on the remote robot
        ./deployer -s [robot].[computer].build --local

#### Launch the robot workspace

- Start launch:

        # start the tmux session, for corresponding robot-computer
        ./deployer -s [robot].[computer].launch.start --local

- Verify launch:

        # enter the docker container on the robot-computer
        docker-join

        # list the available tmux sessions
        tmux list-sessions

        # enter the specified tmux session
        tmux a -t [name-of-session]

- Stop launch:

        # start the tmux session, for corresponding robot-computer
        ./deployer -s [robot].[computer].launch.stop --local

#### Cleanup

- Stop the docker container when turning off the robot:

        ./deployer -s [robot].[computer].docker.stop --local

* * *

### Remote Deploy

All commands below are to be done on the basestation. **DO NOT GO ON THE ROBOT to run the deployer.**

These instructions assume that you have already build the basestation deploy workspace. If not, please go back and [deploy](../documentation/README.md) onto the basestation.

#### Setup the robot workspace

- Navigate to the deploy repo **on the basestation**:

        cd /home/$USER/deploy_ws/src

- Start the docker container on the basestation:

        ./deployer -s basestation.docker.start

- Clone all the submodules & rosinstall repos *(skip over this step if repos are already updated on the basestation)*:
    
        # initialize all the repos to install
        ./deployer -s basestation.init

        # clone all the repos using wstool and submodule update
        ./deployer -s basestation.up

- Transfer the deploy repo from the basestation to the remote robot:

        ./deployer -s [robot].[computer].transfer.to

    - if no remote configuration yaml is setup, notify the [maintainer](../maintainer.md)

- Build the subt docker images on the robot:

        # build the docker images on the robot
        ./deployer -s [robot].[computer].docker.image

#### Build the robot workspace

- Start the docker container on the robot:

        ./deployer -s [robot].[computer].docker.start

- Robot-computer specpfic pre-build steps:

    - **nuc:**
    
            # install thirdparty xsens libraries in the container before building
            ./deployer -s [robot].nuc.rosdep.xsens

- Build the robot workspace:
    
        # this will build the workspace on the remote robot
        ./deployer -s [robot].[computer].build

#### Launch the robot workspace

- Start launch:

  - Desktop launch icons available **on the basestation**:

          # start the tmux session, for corresponding robot-computer
          [robot]_[computer]_start.desktop

      - If not available notify the [maintainer](#who-do-i-talk-to).

  - Manual launch:

          # start the tmux session, for corresponding robot-computer
          ./deployer -s [robot].[computer].launch.start

- Verify launch:

    - Desktop icon launch:
    
      - a tmux session should be started.

  - Manual launch:

          # enter the docker container on the robot-computer
          docker-join

          # list the available tmux sessions
          tmux list-sessions

          # enter the specified tmux session
          tmux a -t [name-of-session]

- Stop launch:
  - Desktop icon launch:

          # stop the tmux session, for corresponding robot-computer
          [robot]_[computer]_stop.desktop

      - If not available notify the [maintainer](#who-do-i-talk-to).

  - Manual launch:

          # start the tmux session, for corresponding robot-computer
          ./deployer -s [robot].[computer].launch.stop --local

#### Cleanup

- Stop the docker container when turning off the robot:

  - Desktop icon launch:

          # stop the tmux session, for corresponding robot-computer
          [robot]_[computer]_stop.desktop

  - Manual launch:

          ./deployer -s [robot].[computer].docker.stop