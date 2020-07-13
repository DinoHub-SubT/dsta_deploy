# Robot Basestation

**Table Of Contents**

[TOC]

The basestation used for robot testing. This deployment should often be in a stable branch.

The robot basestation should not build the planning & state estimation workspaces since it only requires running the basestation gui and perception.

## Robot SSH Keys

**Deploy SSH Keys**
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

## Workspace Build Instructions

  - [Basestation GUI](gui.md)

  - [Basestation Perception](perception.md)

## Basestation TMUX Luanch

The tmux launch requires the basestation GUI & basestation perception workspaces to be built to run.

        # start the basestation tmux session
        tmuxp load tmuxp/basestation.start.yaml

        # stop the basestation tmux session
        tmuxp load tmuxp/basestation.stop.yaml

## Debugging Basestation Helpful Tips

**Enter the docker container:**

      docker-join
    
- you are now in the container

**Building Inside the docker container:**

  - See [top level readme](../README.md#workspaces) for which workspaces exist.
  - `catkin build` inside the workspace. No catkin config is required, all the configurations are already setup.
