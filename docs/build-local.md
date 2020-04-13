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

# Build The Workspaces

TODO

