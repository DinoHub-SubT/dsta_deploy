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

When the user clones the deploy repository, the submodules will not be cloned by default. Therefore, the user must manually clone the submodules.

- The user decides which *group of submodules* to clone.

- Submodules are grouped by algorithm, computer or robot type.

## Required submodules

The user must clone these of submodules.

**Clone the common submodules**

    git submodule update --recursive --init common

## Optional submodules

The user only needs to clone what they need. Choose which group of submodules to clone as listed below.

**Clone the basestation submodules**

    git submodule update --recursive --init basestation

**Clone the ugv submodules**

    git submodule update --recursive --init ugv

**Clone the uav submodules**

    git submodule update --recursive --init uav


* * *

## Tutorial: Updating The Submodules

TODO

* * *

## Removing Submodules

To remove a group of submodules locally *(for example, when the user is done developing with the ugv workspace)*, perform the following:

    # git command structure template:
    #   -> git submodule deinit -f [ group-name ]

    # example, removing all ugv submodules locally
    git submodule deinit -f ugv
