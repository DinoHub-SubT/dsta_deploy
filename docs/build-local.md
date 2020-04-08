# Localhost Deployment Example Walkthrough

Everything will built and run on the host.

- **All dependencies must be installed on the host directly for all workspaces**.

It is **recommended to use docker** (follow the docker tutorial) because the dependencies are maintained. However, you can use whichever method you find most preferable.

This tutorial **will not explain how to install the dependencies**. 
- The user is responsible for installing knowing how to install for each workspace.
- For help in installing dependencies, please see the maintained dockerfiles found in `operations/deploy/docker/dockerfiles/`.
- These dockerfiles maintain the instructions for installing dependencies but in the dockerfile format.
- You can *extract those instructions* from the dockerfile and run the install instructions directly on the host.
- You will need to become familiar with dockerfiles in order to understand how to extract the dependency install instructions.
    - An example *extraction* meaning, taking the `RUN` commands in the dockerfile and running them directly on the host (without the `RUN` keyworkd)

### Prerequisites

1. ROS Melodic *(optional)*

    - please see the official [ROS install instructions](http://wiki.ros.org/melodic/Installation/Ubuntu).

2. All the dependnecies for each workspace must be installed.

    - if you do not know what dependencies are required, please refer to the dockerfiles `operations/deploy/docker/dockerfiles`

# Build The Simulation Workspaces

This build setup assumes you wish to build the repositories locally and  and run their algorithms and simulations.

**Build Common (Required)**

The common catkin workspace contains all common repositories between other workspaces.

    # go to the `common` catkin workspace
    cd ~/deploy_ws/src/common

    # list the catkin profiles available
    catkin profile list

    # set the catkin profile
    catkin profile set system76-pc

    # build the catkin workspace
    catkin build

**Build Basestation (Optional) **

The basestation catkin workspace contains all basestation related repositories (example GUI control).

    # go to the `basestation` catkin workspace
    cd ~/deploy_ws/src/basestation

    # list the catkin profiles available
    catkin profile list

    # set the catkin profile
    catkin profile set system76-pc

    # build the catkin workspace
    catkin build

**Build UAV Simulation (Optional)**

The uav simulation catkin workspace contains all repositories related in running the uav in simulation.

    # go to the `common` catkin workspace
    cd ~/deploy_ws/src/uav/sim

    # list the catkin profiles available
    catkin profile list

    # set the catkin profile
    catkin profile set system76-pc

    # build the catkin workspace
    catkin build

**Build UGV: Planning-PC (Optional) **

The ugv planning-pc catkin workspace contains all repositories related in running the ugv's planning-pc in simulation.

    # go to the `ppc` catkin workspace
    cd ~/deploy_ws/src/ugv/planning-pc

    # list the catkin profiles available
    catkin profile list

    # set the catkin profile
    catkin profile set system76-pc

    # build the catkin workspace
    catkin build

**Build UGV: NUC (Optional) **

The ugv nuc catkin workspace contains all repositories related in running the ugv's nuc in simulation.

    # go to the `ppc` catkin workspace
    cd ~/deploy_ws/src/ugv/nuc

    # list the catkin profiles available
    catkin profile list

    # set the catkin profile
    catkin profile set system76-pc

    # build the catkin workspace
    catkin build

* * *

# Build The Robot Workspaces

This build setup assumes you wish to build the repositories on respective robots and run their algorithms and hardware repositories.

TODO