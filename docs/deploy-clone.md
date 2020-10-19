# Deploy Workspaces

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

**List of repositories with permission restrictions:**

- `ugv/slam/devel/loam_velodyne_16`
- `ugv/slam/devel/online_pose_graph`
- `ugv/slam/devel/velodyne_driver_16_32`
- `uav/slam/loam_velodyne_16`
- `uav/slam/online_pose_graph`
- `uav/slam/velodyne_driver_16_32`

**List of repositories with more restricted permission:**

- `ugv/slam/robot/laser_odometry`

Please notify the maintainer if you need to be making changes to the robot's `LOAM` package.

## Layout

    deploy (meta-repo) [submodule]

        operations [submodule]
            * -- contains all operations configurations such as: azure, docker, ansible installs, deployment scripts, etc.
            deploy/
                azurebooks
                    * -- contains terraform configurations files for Azure resources setup.
                deployerbooks
                    * -- contains deployer yaml configuration files for automating system deployment
                deployer
                    * -- deployer execution scripts
                docker
                    * -- contains dockerfiles, docker-compose for setting docker images & containers on different systems
                robotbooks
                    * -- contains ansible scripts for automating system package & configuration installs on different systems
                scenarios
                    * -- environment files for exporting environment variables used for configuring docker-compose & deployer scripts
            field_testing
                *-- snapshot logfiles created during field testing
            utils
                sysadmin
                    *-- helpful all-purpose utility scripts.
                    *-- Scripts added here can be used from anywhere.
                robot-configurations
                    *-- passive configuration reference files
                ugv_setup
                    *-- robot configuration scripts (ugv specific)

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

## Deploy Submodules on Localhost

This tutorial will show you how to interact with deployer's git tools.

- Please use `TAB` as autocompletion to `subt git [TAB]`, to see more help information about the deployer git commands.
- Alternatively, you can manually clone the submodules if you do not want to use the deployer.

### Clone the submodules

This will clone all the base submodules, for running the system on localhost.

    subt git clone localhost

If you wish to clone specific projects:

    # preview which submodules will be cloned
    subt git clone localhost -p

    # clone only a specific group
    subt git clone localhost.common

    # clone multiple groups
    subt git clone localhost.common localhost.basestation

- Submodules clone as `DETACHED HEAD`. Make sure to always checkout to a branch after its cloned.

- The `clone`, will **RESET** your submodule's checked-out branch to the origin commit HASH.

### Reset the submodules

This will reset all the submodules, to their `DETACHED HEAD` commit HASH as pushed on origin.

    subt git reset localhost

If you wish to reset specific projects:

    # preview which submodules will be reset
    subt git reset localhost -p

    # reset only a specific group
    subt git reset localhost.common

    # reset multiple groups
    subt git reset localhost.common localhost.basestation

- Submodules reset as `DETACHED HEAD`. Make sure to always checkout to a branch after its cloned.

- The `reset`, will **RESET** your submodule's checked-out branch to the origin commit HASH.

- The `reset` is very similar to clone. Use `reset` when the submodules are already cloned and `clone` when the submodules are empty.

### Pull the submodules

This will pull the submodule's updates, when the submodules are checked-out at a specific branch.

    subt git pull localhost

If you wish to pull specific projects:

    # preview all the available projects available to clean.
    subt git pull localhost -p

    # pull only a specific group
    subt git pull localhost.common

    # pull multiple groups
    subt git pull localhost.common localhost.basestation

### Clean the submodules

This cleans all the base submodules from any uncommitted changes.

    subt git clean localhost

If you wish to clean specific projects:

    # preview all the available projects available to clean.
    subt git clean localhost -p

    # clean only a specific group
    subt git clean localhost.common

    # clean multiple groups
    subt git clean localhost.common localhost.basestation

### Remove the submodules

Remove the submodules locally. It is equivalent to `git submodule --deinit /path/to/submodule`

    subt git rm localhost

If you wish to remove specific projects:

    # preview which submodules will be removed
    subt git rm localhost -p

    # removes only a specific group
    subt git rm localhost.common

    # removes multiple groups
    subt git rm localhost.common localhost.basestation

### Sync the submodules

This will sync all the submodules local branches with their remotes.

    subt git sync

If you wish to sync specific projects:

    # example, pull common project
    subt git sync common

    # example, multi repo pull
    subt git sync common simulation

- The `sync` does a `fetch --all` and `reset --hard origin` for all submodules, recursively.
- The `reset --hard` makes sure the local branch is up-to-date with its respective remote branch.
- Use `sync` when you want to keep your submodules up-to-date with origin's changes.

### Show the status of the submodules

This will show the status of all the submodules.

    subt git status

If you wish to sync specific projects:

    # example, pull common project
    subt git status common

    # example, pull common project, as a table format
    subt git status common -table

    # example, multi repo pull
    subt git status common simulation -table

* * *

## More References

### Checkout New Submodule Branch (experimental)

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # create a new branch in all submodules
    ./deployer -s git.create -e branch=myuserid/feature-a

    # (example) verify the git status in simulation
    cd ~/deploy_ws/src/simulation
    git status

### Checkout An Existing Submodule Branch (experimental)

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # checkout the existing develop branch in all submodules
    ./deployer -s git.co -e branch=develop

    # (example) verify the git status in simulation
    cd ~/deploy_ws/src/simulation
    git status

### Regex Matcher, Auto-Completion

The `deployer` does a `regex` match, autocompletion for arguments.

So you do not need to give the full deployer arguments, you can just give a partial argument.

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # Full command: clone the core repositories
    ./deployer -s local.git.submodule.clone.base.basestation

    # regex match of the above command
    ./deployer -s git.clone.basestation

- Both commands perform the same.
- Use the preview `-p` option to see what deployer arguments are available.

### Preview and Verbose

The `deployer` has a preview & verbose option.

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # preview all available clone projects
    ./deployer -s git.clone -p

    # preview & verbose all available clone projects
    ./deployer -s git.clone.basestation -p -v

- The preview options just shows (does not execute) what are all the available deployer commands, from a certain prefix.

- The verbose options shows exactly what bash commands are to be run.