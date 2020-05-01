# Installing The Repositories

All the SubT repositories are *maintained* in the deploy workspace as [git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules).

**About SubT Workspaces**

The SubT repositories are grouped as the following workspaces:

`common`:

- found in `deploy_ws/src/common`

- contains repositories common between all other repositories (basestation, uav, ugv)
  
`basestation`:

- found in `deploy_ws/src/basestation`

- contains all repositories for the subt basestation control


`ugv`:

- found in `deploy_ws/src/ugv`

- contains all simualation and hardware repositories for the subt ground robots
  
`uav`:

- found in `deploy_ws/src/uav`

- contains all simualation and hardware repositories for the subt drones

* * *

## Tutorial: Install The Submodules

When you clone the deploy repository, the submodules will not be cloned by default. Therefore, you must manually clone the submodules.

- You must decide which *group of submodules* to clone.

- Submodules are grouped by algorithm, computer or robot type.

**If you have any errors cloning the submodules, notify the maintainer.**

- You might need to be given permissions to clone the repositories.

## Required submodules

These are the required submodules that must be cloned.

**Clone the operations submodules**

    # clone the submodules
    git submodule update --recursive --init operations

**Clone the common submodules**

    # clone the submodules
    git submodule update --recursive --init common

    # check the git status (please do this step and wait until command is completed)
    git status

    # checkout the git-lfs files
    cd common/communication_manager
    git lfs fetch
    git lfs pull

## Optional submodules

The user only needs to clone what they need. Choose which group of submodules to clone as listed below.

**Clone the basestation submodules**

    # clone the submodules
    git submodule update --recursive --init basestation

    # check the git status (please do this step and wait until command is completed)
    git status

**Clone the ugv submodules**

    # clone the submodules
    git submodule update --recursive --init ugv

    # check the git status (please do this step and wait until command is completed)
    git status

    # checkout the git-lfs files
    cd ugv/sim/subt_gazebo
    git lfs fetch
    git lfs pull

**Clone the uav submodules**

    # clone the submodules
    git submodule update --recursive --init uav

    # check the git status (please do this step and wait until command is completed)
    git status

* * *


## Removing Submodules

To remove a group of submodules locally *(for example, when the user is done developing with the ugv workspace)*, perform the following:

    # git command structure template:
    #   -> git submodule deinit -f [ group-name ]

    # example, removing all ugv submodules locally
    git submodule deinit -f ugv
