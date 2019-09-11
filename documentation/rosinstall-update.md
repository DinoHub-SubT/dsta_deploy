# Update Rosinstalls & Submodules

**Table Of Contents**

[TOC]

## Overview

- The deploy repo **does not** git commit any of the algorithm repositories directly.
  - Instead, the deploy repo maintains the repositories with rosinstalls & submodules.
  - The deploy repo maintains the workspace catkin profile settings for building.
    - See `planning/.catkin_tools/profiles/default/config.yaml` as an example.
  
- Once testing & development are completed, the user needs to update the rosinstalls or submodules with the commit hash of the updated repos. This way, the deploy repos is always up-to-date with the most stable versions of all the repositories.

    Update Type       | Workspace
    ------------- | -------------
    rosinstall | basestation, planning, state estimation
    submodules | perception

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

* * *

## How To Update the deploy rosinstalls

- Navigate to the deploy repo **on the basestation**:

        cd /home/$USER/deploy_ws/src/

- Update the rosinstall:

    - edit any *rosinstall* found in `rosinstall/`

    - update the repo's `commit hash` with either a branch name or a different commit hash.

        - a `commit hash` is preferred, in order to maintain specific commit versions.

- Use `wstool` to update the repo in the deploy workspace:

    - Slower, full update:

            # start the docker container on the basestation
            ./deployer -s basestation.docker.start

            # update all the repositories on the basestation, in the docker container
            ./deployer -s basestation.up

    - Faster update:

            # start the docker container on the basestation
            ./deployer -s basestation.docker.start

            # enter the docker container on the basestation
            docker-join

            # navigate to the deploy workspace, in the docker container, to the repository to update
            cd ~/deploy_ws/src/path/to/repository

            # update the repo
            wstool up

* * *

## How To Update the deploy submodules

- Navigate to the deploy repo **on the basestation**:

        cd /home/$USER/deploy_ws/src

- Update the submodules:

        git submodule update --init --recursive

- Submodule(s) fail to update *example fix*:

        # navigate to the submodule
        cd perception/object_detection
        
        git checkout master
        git pull origin master

[[top]](#markdown-header-deployer)