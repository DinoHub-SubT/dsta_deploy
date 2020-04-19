# Build With Azure Walkthrough

This tutorial will outline how to setup every workspace on the Azure VM.

- Everything will build and run on the Azure VMs.

- Please only follow the instructions that match your chosen setup.

You can install the workspaces directly on the VM or using docker.

  - The below instructions *(recommended method)* will follow the docker setup.

Please make sure you have completed the *Build Azure Prerequisites* instructions.

## Quick Start

- Explains how to install workspace dependencies in docker images and build catkin workspaces in docker containers on the remote VM.
- This method is the *recommended method to use* because it is the most straight-forward for repository access and debugging in docker containers.

### 1. Verify Azure VM Dependencies

Verify you have all the third-party operations tools installed correctly (if not already done so):

        # ssh into your VM (if not already done so), please change the below command to match your VM ssh access
        ssh azure.ugv1

        # verify docker
        docker --version

        # verify docker-compose
        docker-compose -v

        # verify docker-compose shows the help usage message
        docker-compose-wrapper --help
        
        # verify deployer script shows the help usage message
        cd ~/deploy_ws/src
        ./deployer --help

        # exit the Azure VM (return to localhost)
        exit

### 2. Azure Docker Setup

Setup the docker images and containers by following any of the docker build tutorials:

Tutorials at:

  - **Basestation:** [`docs/azure-docker-basestation-setup.md`](azure-docker-basestation-setup.md)
  - **UGV:** [`docs/azure-docker-ugv-setup.md`](azure-docker-ugv-setup.md)
  - **UAV:** [`docs/azure-docker-uav-setup.md`](azure-docker-uav-setup.md)
  
### 3. Build Catkin Workspace on Azure VMs

Build the workspace by following any of the docker build tutorials:

  - **Basestation:** [`docs/catkin-basestation.md`](catkin-basestation.md)
  - **UGV:** [`docs/catkin-ugv.md`](catkin-ugv.md)
  - **UAV:** [`docs/catkin-uav.md`](catkin-uav.md)


### 4. Summary

You should now have a built `SubT` workspace on a Azure VM.

- Please notify the maintainer if any of the tutorial steps did not succeed.

Please go back to [`build-tutorial`](build-tutorial.md#Summary) for summary comments.