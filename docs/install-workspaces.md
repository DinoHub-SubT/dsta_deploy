# Deploy Workspaces

The `deploy` workspace *maintains* all the `SubT` repositories as [git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules).

- Read the [deploy wiki page](https://bitbucket.org/cmusubt/deploy/wiki/tutorials/submodules) for a simple submodule tutorial review.

## About Deploy Layout

The deploy repos maintains all `SubT` repositories as nested submodules.

Submodules has many advantages:

- submodules provide the ability to maintain a snapshot of repositories.
- submodule allow for easy, isolate versioning.
- intermediate submodule levels allow for easy, isolated group versioning.
- submodules does not require learning a new toolset and provides user feedback from `git status`.

**Commit Levels**

The deploy repo is not flat hierarchy of submodules. It maintains a **3-commit level** groups of submodules as such:

    deploy (meta-repo) [submodule]
        intermediate-dir (meta-repo) [submodule]
            module-dir  [submodule]

**Layout**

The full layout, with descriptions, is as such:

    deploy (meta-repo) [submodule]

        operations [submodule]
            * -- contains all operations configurations such as: azure, docker, ansible installs, deployment scripts, etc.

        common [submodule]
            * -- contains repositories that are common between basestation, ugv, uav repos
            * -- contains common ros messages

        simulation [submodule]
            * -- contains all repositories related to simulation launch of gazebo: world environments, robots, etc.
            * -- contains third-party darpa's gazebo world setup.
            * -- clone only on simulation systems, not on robots.
            darpa
            ignition (potentially)

        subt_launch [submodule]
            * -- central launch repository.
            * -- contains a centralized location for different robot params
            * -- creates groups of top-level launch scenarios, to handle different robot & system setups.

        basestation [submodule]
            * -- contains all repositories related to the basestation GUI

        perception [submodule]
            * -- contains all repositories related to the perception object detection

        ugv [submodule]
            * -- contains all repositories related to ground robots
            * -- sub-grouped into folders, named by the ground robot "computers" (planning-pc, nuc)
            * -- hardware directory maintains all hardware related ugv repositories. clone only on robots, not in simulation.
            * -- slam is a separate folder because of user permission restrictions
            * -- some repositories will be built outside of docker (please refer to operations maintainer for details)
            planning-pc
                repo1 [submodule]
                repo2 [submodule]
                ...
            nuc
                repo1 [submodule]
                repo2 [submodule]
                ...
            slam (example: laser only)
                * - has user permission restrictions, cannot be cloned by everyone
                laser_odometry (loam_velodyne_16, receive_xsens, velodyne_driver_16) will be outside docker
            hardware
                *- should only be cloned when testing on robots
                *- uses catkin profile to switch between hardware repositories to build

        uav [submodule]
            * -- contains all repositories related to drone robots
            * -- core algorithm repositories are found in the `uav/core` directory
            * -- hardware directory maintains all hardware related uav repositories. clone only on robots, not in simulation.
            * -- slam is a separate folder because of user permission restrictions
            core
                repo1 [submodule]
                repo2 [submodule]
                ...
            hardware
                *- should only be cloned when testing on robots
            slam (example: camera, laser)
                * - has user permission restrictions, cannot be cloned by everyone

**Summary:**

- The *deploy repository* is a 3-level commit layout
- You will always need to do at least 3 commits, when making changes in the lowest level submodule.
- Leverage the ability to create branches in intermediate-level submodules.
    - git branches in the intermediate-levels will maintain independent development workflow and versioning.

* * *

## Tutorial: Localhost Deploy Workspace Setup

### About

This tutorial will walk you through on how to manually clone all the submodules.

- These steps can be automated, however it will be good practice to try it manually in order to become familiar with submodules and the deploy repository layout.

**Basic Information**

- When you clone the deploy repository, the submodules will not be cloned by default.

    - You must manually clone the submodules.

- You must decide which *submodule level groups* to clone.

- Some submodule have user permission restrictions.
    - You do not need to clone these repositories, unless you are doing active development on them.
    - Please notifier the maintainer to give you permission to access the restricted repositories.

**List of restricted permission repositories:**

    ugv/slam/laser_odometry

- If you have any errors cloning the submodules, notify the maintainer.

### Required submodules

These are the required submodules that must be cloned by every user.

Please perform the following:

**Clone the operations submodules**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # clone the submodules
    git submodule update --recursive --init operations

**Clone the common submodules**

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

**Clone the central launch submodule**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # clone the submodules
    git submodule update --recursive --init subt_launch

### Optional Submodules

These are optional submodules, that are to be cloned depending on the user's development.

Please perform any of the following:

**Basestation**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # clone the submodules
    git submodule update --recursive --init basestation

    # check the git status (please do this step and wait until command is completed)
    git status

**UGV (ground robots)**

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
    cd uav/sim/subt_gazebo
    git lfs fetch
    git lfs pull

* * *

## Fixing Submodules Issues

**sync submodule with remote:**

    git submodule sync [ submodule name ]

## Removing Submodules

To remove a submodule-level, use the `deinit` command.

**Command template:**

    git submodule deinit -f [ submodule name ]

**Example, remove entire intermediate-level:**

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # example, removing all ugv submodules locally
    git submodule deinit -f ugv

**Example, remove part of an intermediate-level:**

    # go to the ugv, intermediate-level submodule
    # -- you must be inside the intermediate-level directory:
    cd ~/deploy_ws/src/ugv

    # example, removing all ugv::hardware submodules locally
    git submodule deinit -f hardware
