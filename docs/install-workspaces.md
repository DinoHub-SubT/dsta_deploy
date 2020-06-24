# Deploy Workspaces

The `deploy` workspace *maintains* all the `SubT` repositories as nested [git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules).

- Read the [deploy wiki page](https://bitbucket.org/cmusubt/deploy/wiki/tutorials/submodules) for a simple submodule tutorial review.

Submodules has many advantages:

- submodules provide the ability to maintain a snapshot of repositories.
- submodule allow for easy, isolate versioning.
- intermediate submodule levels allow for easy, isolated group versioning.
- submodules does not require learning a new toolset and provides user feedback from `git status`.

## Layout


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
                *- uses catkin profile to set which hardware repositories to build

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


## Terminology

**Commit Levels**

The deploy repo maintains a **3-commit level** group of submodules:

    deploy (meta-repo) [submodule]
        intermediate-dir (meta-repo) [submodule]
            module-dir  [submodule]

**Meta-repo (submodule)**

  - the top level is called a *meta-repo*
  - the *meta-repo* is a submodule
  - it will maintain a working version of the entire workspace
  - the meta-repo level submodule repo will group working versions of **intermediate-repos**
  - **an update to the meta-repo only requires one commit**: a commit is added at the *meta-repo* level.

**Intermediate level directory (submodule)**

  - the *intermediate-group-repo* is an intermediate level directory inside the *meta-repo*
  - the intermediate level directory is a submodule
  - the intermediate level submodule repo will group working versions of **module-repos**
  - **an update to the intermediate level requires 2 commits:** a commit at the *intermediate level* and a commit at the *meta-repo* level.

**Module-repo (submodule)**

  - the *module-repo* is the lowest level directory repo, it is inside a *intermediate level*
  - the *module-repo* are the actual algorithms being developed
  - **an update to the module-repo level requires 3 commits:** a commit at the module-repo, a commit at the *intermediate level* and a commit at the *meta-repo* level.

**Directory (non-submodule)**

  - just a directory in any level
  - an update to a directory commit depends on which `level` it is located in

**Summary:**

- The *deploy repository* is a 3-level commit layout
- You will always need to do at least 3 commits, when making changes in the lowest level submodule.
- Utilize the ability to create branches in intermediate-level submodules.
    - git branches at the intermediate-levels will maintain independent development workflow and versioning.

* * *

## Tutorial: Localhost Deploy Workspace Setup


This tutorial will walk you through on how to manually clone all the submodules.

- These steps can be automated, however it will be good practice to try it out manually once in order to become familiar with submodules and the deploy repository layout.

If you have any errors cloning the submodules, notify the maintainer.

**Basic Information**

- When you clone the deploy repository, the submodules will not be cloned by default.

    - You must manually clone the submodules.

- You must decide which *submodule level* to clone.

- Some submodules have user permission restrictions.
    - You do not need to clone these repositories, unless you are doing active development on them.
    - **If you need these repos, please notifier the maintainer to give you permission to access the restricted repositories.**

**List of restricted permission repositories:**

- `ugv/slam/laser_odometry`
- `ugv/slam/online_pose_graph`


### Required submodules

These are the required submodules that must be cloned by every user.

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
    cd simulation/subt_gazebo
    git lfs fetch
    git lfs pull

* * *

## Updating Submodules

**Re-clone submodules:**

    # get the latest update
    git pull origin [feature branch name]

    # re-clone the submodules
    git submodule update --recursive --init [ submodule name ]

**Sync submodule with remote:**

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

**Example, partly remove modules in an intermediate-level:**

To remove only the ugv hardware repos:

    # go to the ugv, intermediate-level submodule
    # -- you must be inside the intermediate-level directory:
    cd ~/deploy_ws/src/ugv

    # example, removing all ugv::hardware submodules locally
    git submodule deinit -f hardware
