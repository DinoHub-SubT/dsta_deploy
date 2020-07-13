# Local Basestation

**Table Of Contents**

[TOC]

The basestation used for local development.

The development basestation should be able to build all of the workspaces.

## Workspace Build Instructions

  - [Basestation GUI](gui.md)

  - [Basestation Perception](perception.md)

  - [Basestation Planning](planning.md)

  - [Basestation State Estimation](state-estimation.md)

## Basestation TMUX Luanch

The tmux launch lauches the basestation GUI & basestation perception workspaces only.

        # start the basestation tmux session
        tmuxp load tmuxp/basestation.start.yaml

        # stop the basestation tmux session
        tmuxp load tmuxp/basestation.stop.yaml

You can add more tmux setups to launch other workspaces locally.

## Debugging Basestation Helpful Tips

**Enter the docker container:**

      docker-join
    
- you are now in the container

**Building Inside the docker container:**

  - See [top level readme](../README.md#workspaces) for which workspaces exist.
  - `catkin build` inside the workspace. No catkin config is required, all the configurations are already setup.
