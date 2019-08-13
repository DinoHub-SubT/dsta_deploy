# Basestation Perception

**Table Of Contents**

[TOC]

## Overview

Currently, perception docker image is using `cuda 9`.

## Basestation Perception Build & Launch

1. **Clone the Basestation Perception Workspace**

        # Initializes the basestation rosinstall
        ./deployer -s basestation.rosinstall.perception

        # wstool updates
        ./deployer -s basestation.up.perception

2. **Download `nv-tensorrt` from perceptron**

        cd ~/deploy_ws/src/docker/
        scp -v /project/subt/data/docker-deploy/nv-tensorrt-repo-ubuntu1604-cuda9.0-ga-trt4.0.1.6-20180612_1-1_amd64.deb .

    Setup a cluster perceptron account, if you do not already have one. Notify the [maintainer](../maintainer.md) about getting one.
  
3. **Build the docker image**

        ./deployer -s basestation.docker.perception.image

4. **Start the docker container**

        ./deployer -s basestation.docker.perception.start

5. **Build the Basestation Workspace**

        ./deployer -s basestation.perception.build

6. **Launch the Basestation Workspace**

        ./deployer -s basestation.perception.launch.start

7. **Source the workspace**

        echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
        echo "export ROS_HOSTNAME=[hostname]" >> ~/.bashrc
        
        # source launch workspace devel path
        echo "source ~/deploy_ws/devel/basestation/launch/setup.bash" >> ~/.bashrc

8. **Stop the Basestation Workspace Launch**

        ./deployer -s basestation.perception.launch.stop

9.  **Stop the Docker Container**

        ./deployer -s basestation.docker.perception.stop
