
# New System Deployment

There are different types of deployments available.

- [Local Development](#markdown-header-local-development)
- [Basestation](#markdown-header-basestation)
- [Robot](#markdown-header-robot)

Each deployment has its own catkin workspace and launch configuration preconfigured & setup.

[[top]](#markdown-header-deployer)

## Local Development

### Install Docker on local development

Follow the install docker [instructions](#markdown-header-install-docker)

   - Follow the *Install Nvidia Docker* if you have a Nvidia GPU.

Once installed once, you do not need to install again.

### Deploy on the local development

Follow the [local development workstation](documentation/new_system_deployment_development_direct.md) instructions.

[[top]](#markdown-header-deployer)


## Basestation

### Install Docker on the basestation

Follow the install docker [instructions](#markdown-header-install-docker)

   - **Do not** follow the *Install Nvidia Docker*. Only follow the basic docker install instructions.

Once installed once, you do not need to install again.

### Deploy on the basestation

Follow the [basestation deployment](documentation/new_system_deployment_basestation_direct.md) instructions.

[[top]](#markdown-header-deployer)

## Robot

### Install Docker on the robot

Follow the [docker install](#markdown-header-install-docker) instructions:

  - **Do not** follow the *Install Nvidia Docker*. Only follow the basic docker install instructions.

  - internet access required.

Once installed once, you do not need to install again.

### Deploy on the robot

Follow the [robot deployment](documentation/new_system_deployment_robot_direct.md) instructions.

[[top]](#markdown-header-deployer)

* * *

# Updating Existing Deployment

Updating an existing deployment *should always* be done from the basestation, **not directly on the robot**.

## Choosing the Deployment

- **Update the workspace repositories**
    1. Maintain the deploy_ws git branch:

          - a new update should start from the *develop branch*.
          - remember to always pull from develop for any new changes.

    2. Update the rosinstalls and the submodules:
     
        - [rosinstalls](#markdown-header-update-the-deploy-rosinstalls)
        - [submodules](#markdown-header-update-the-deploy-submodules)

    3. Build one of the following the workspace:

        - [local development](documentation/new_system_deployment_development_direct.md)
        - [basestation](documentation/new_system_deployment_basestation_direct.md)
        - [robot](documentation/new_system_deployment_robot_direct.md)

    - If a new workspace is required to be added, notify the [maintainer](../../maintainer.md).

- **Update the docker image**
    
    - only update the docker image if a new system dependency is needed.
    - [planning dockerfiles](#markdown-header-planning-dockerfiles)
    - [perception dockerfiles](#markdown-header-perception-dockerfiles)

[[top]](#markdown-header-deployer)

* * *
