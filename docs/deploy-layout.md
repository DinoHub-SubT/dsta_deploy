# Deploy Workspaces

## Why Submodules

The `deploy` workspace *maintains* all the `SubT` repositories as nested [git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules).

**Submodules have many advantages:**

- provides the ability to maintain a snapshot of repositories.
- allows for isolated versioning.
- intermediate submodule levels allow for isolated group versioning.
- submodules status is knowing using `git status`.

**When you clone the deploy repository, the submodules will not be cloned by default**.

- You must also clone the submodules.

- You must decide which *submodule levels* to clone.

## Repository Permissions

Some submodules have user permission restrictions.

  - You do not need to clone these repositories, unless you are doing active development on them.
  - If you need these repos, then please notify the maintainer to give you permission to access the restricted repositories.

**List of repositories with very restricted permission:**

- `ugv/slam/*`
- `uav/slam/*`

Please notify the maintainer if you need to be making changes to the robot's `slam` packages.

## Layout

```text
deploy (meta-repo) [submodule]

    docs
        *-- various markdown readmes

    operations [submodule]
        * -- contains all operations configurations such as: azure, docker, ansible installs, deployment scripts, etc.
        * ansiblebooks
            * -- contains ansible scripts for automating system package & configuration installs on different systems
        azurebooks
            * -- contains terraform configurations files for Azure resources setup.
        deployerbooks
            * -- contains deployer yaml configuration files for automating system deployment
        deployer
            * -- deployer execution scripts
        docker
            * -- contains dockerfiles, docker-compose for setting docker images & containers on different systems
        scenarios
            * -- environment files for exporting environment variables used for configuring docker-compose & deployer scripts
        field_testing
            *-- snapshot logfiles created during field testing
        robot_snapshot
            *-- a snapshot of robot configurations (such as services, networking, etc).
            ugv_setup
                *-- robot configuration scripts (ugv specific)
        scripts
            *-- common used scripts, all scripts are added to `PATH`

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
```

## Terminology

The deploy repo maintains a **2-commit level** group of submodules (example):

```text
deploy_ws (1-level) [meta-repo, submodule]
    communication_manager (2-level) [submodule]
```

You will very often find yourself making 2 commits.
  - During unit testing we can easily switch between 2-levels
  - Commit to 1-level only when the feature is fully tested on robots.

Pull Requests are preferred, however pushing directly to `develop` is OK even if it breaks. As developers, only push to `develop` as the main branch, the core maintainers will handle pushing to `stable` and `master`.

### 1-level

  - the 1-level is called a deploy repo, located as `~/deploy_ws/src`.
  - it is a submodule, used as a *meta-repo*
  - it will maintain a working version of the entire workspace, weekly testing tags are added at this level.

## 2-level

  - the 2-level is a the main algorithm repo, inside the 2-level
  - the *2-level* are the actual algorithms being developed
  - when updating 2-level, you will need another commit in 2-level and 1-level. commit to 1-level when the 2-level is fully tested on robot hardware.
