# Deploy Workspaces Extra

## Project Clone Deploy Workspace

If you are missing packages with the base clone, you should then clone the specific projects.

**Operations**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/
    ./deployer -s git.clone.operations

**Common**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/
    ./deployer -s git.clone.common

**Central Launch**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/
    ./deployer -s git.clone.subt_launch

**Basestation**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/
    ./deployer -s git.clone.basestation

**UGV (ground robots)**

Shorter Version

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    ./deployer -s git.clone.ugv

Longer Version

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # (required) clone the core repositories (planning-pc, nuc)
    ./deployer -s git.clone.ugv.ppc
    ./deployer -s git.clone.ugv.nuc

    # (optional) clone the ugv hardware repositories
    ./deployer -s git.clone.ugv.hardware

    # (optional) clone the ugv slam repositories
    ./deployer -s git.clone.ugv.slam

**UAV (drone robots)**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # (required) clone the core repositories (core)
    ./deployer -s git.clone.uav.core

    # (optional) clone the hardware repositories
    ./deployer -s git.clone.uav.hardware

    # (optional) clone the slam repositories -- only if you have user permissions
    ./deployer -s git.clone.uav.slam

**Perception (object detection)**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # (required) clone the core repositories
    ./deployer -s git.clone.perception


**Simulation (gazebo, ignition)**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # (required) clone the core repositories
    ./deployer -s git.clone.simulation

* * *

## Manual Clone Deploy Workspace (for reference)

This tutorial will walk you through on how to manually clone the deploy repo and all its submodules.

Please use the `deployer`, previous instructions to clone. However, if you wish to know how to clone manually, then follow this tutorial.

**Operations (required)**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # clone the submodules
    git submodule update --recursive --init operations

**Common**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # clone the submodules
    git submodule update --recursive --init common

    # check the git status (please do this step and wait until command is completed)
    git status

    # checkout the git-lfs files
    cd common/communication_manager
    git lfs fetch
    git lfs pull

**Central Launch**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # clone the submodules
    git submodule update --recursive --init subt_launch

**Basestation**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # clone the submodules
    git submodule update --recursive --init basestation

    # check the git status (please do this step and wait until command is completed)
    git status

**UGV**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # shallow clone the ugv 'intermediate-level' submodule
    git submodule update --init ugv

    # go inside the ugv, 'intermediate-level' submodule
    cd ~/deploy_ws/src/ugv

    # (required) clone the core repositories
    git submodule update --init --recursive planning-pc/
    git submodule update --init --recursive nuc/

    # (optional) clone the hardware repositories
    git submodule update --init --recursive hardware

    # (optional) clone the slam repositories
    # -- user permission restrictions, only clone if you have permissions to do so.
    git submodule update --init --recursive slam/laser_odometry

    # check the git status (please do this step and wait until command is completed)
    git status

**UAV (drone robots)**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # shallow clone the uav 'intermediate-level' submodule
    git submodule update --init uav

    # go inside the ugv, 'intermediate-level' submodule
    cd ~/deploy_ws/src/uav

    # (required) clone the core repositories
    git submodule update --init --recursive core

    # (optional) clone the hardware repositories
    # -- clone only on the ground robot
    git submodule update --init --recursive hardware

    # check the git status (please do this step and wait until command is completed)
    git status

**Perception (object detection)**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # clone the submodules
    git submodule update --recursive --init perception

    # check the git status (please do this step and wait until command is completed)
    git status

**Simulation (gazebo, ignition)**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # clone the submodules
    git submodule update --recursive --init simulation

    # check the git status (please do this step and wait until command is completed)
    git status

    # checkout the git-lfs files
    cd simulation/subt_gazebo
    git lfs fetch
    git lfs pull

