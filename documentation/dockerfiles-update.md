# Dockerfiles

**Table Of Contents**

[TOC]

* * *

## Overview

Updating docker images requires changing dockerfiles.

The `dockerfiles` for creating docker images are located in `docker/dockerfiles/subt/local/`

Scripts to build, create, run docker images and containers are found in `docker/scripts/`

Docker container options are maintained in `docker/env/field/robot/`

- Additional default docker run commands can be found in `docker/scripts/run.bash`

[[top]](#markdown-header-deployer)

* * *

## Planning

### Update the dockerfiles

All planning workspace dockerfiles are found in:

        # dockerfiles for non-planning workspace dependencies
        docker/dockerfiles/subt/local/osrf/
        
        # dockerfiles for planning workspace only dependencies
        docker/dockerfiles/subt/local/subt/

- **General dependencies** should be added to `docker/dockerfiles/subt/local/osrf/ros_0.1.dockerfile`

- **Repository dependencies** should be added to `docker/dockerfiles/subt/local/subt/ros_0.1.dockerfile`

  - If you are not sure where to add a new dependency, add it anywhere in the `docker/dockerfiles/subt/local/subt/ros_0.1.dockerfile`

### Build the Docker Images

Once added, you will need to rebuild docker images on which robot or computers require the change.

#### Basestation Docker Image Build

- Remove the previous docker container:
  
        ./deployer -s basestation.docker.gui.remove

- Build the docker image:

        ./deployer -s basestation.docker.gui.image

- internet access required.

#### Robot Docker Image Build

- Remove the previous docker container:
  
        ./deployer -s [robot].[computer].docker.remove

- Transfer the dockerfile:

        ./deployer -s [robot].[computer].transfer.to

- Build the docker image on robot:

        ./deployer -s [robot].[computer].docker.image

- internet access required.
- remember, you must always transfer any changes made to the remote system.

Once the docker image is updated, rebuild your workspace and launch.

[[top]](#markdown-header-deployer)

* * *

## Perception

*Notice:* Perception docker image is currently only enable locally or on basestation, not on robot xavier.

All perception workspace dockerfiles are found in:

        # dockerfiles for perception workspace dependencies
        docker/dockerfiles/subt/local/tensorflow/

Any one of those dockerfiles can be changed to add a dependency.

  - If you are not sure where to add a new dependency, add it anywhere in the `docker/dockerfiles/subt/local/tensorflow/ros_0.1-binaries.dockerfile`

Once added, you will need to rebuild docker images on which robot or computers require the change.

### Basestation Docker Image Build

- Remove the previous docker container:
  
      ./deployer -s basestation.docker.gui.remove

- Build the docker image:

      ./deployer -s basestation.docker.perception.image

Once the docker image is updated, rebuild your workspace and launch.

[[top]](#markdown-header-deployer)
