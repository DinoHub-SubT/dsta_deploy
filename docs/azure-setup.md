
# Azure Cloud Infrastructure Setup

**Please follow this tutorial only if you are planning on using the Azure Cloud resource.**

[TOC]

This tutorial will outline how to setup:
- Azure VM infrastructure.
- How to install the docker setup on the remote VMs.

You have the option of building the workspaces directly on the VM or in the docker containers.

- This tutorial will follow the docker setup.

## 1. Create the Cloud Infrastructure

**Tutorial at:** [`operations/deploy/azurebooks/README.md`](../operations/deploy/azurebooks/README.md)

- Creates an example azure infrastructure with VMs, virtual networking, VPN in the `SubT` resource group.
- Gives an introduction tutorial on using the `terraform` tools for maintaining the Azure resources.
- Sets up VPN access.
- Sets up remote desktop access.

## 2. Verify the Cloud Infrastructure

**Verify** the cloud infrastructure was created. Go to the [portal.azure.com](https://portal.azure.com/#home) and search for your infrastructure setup.

- In the `SubT` resource group, search for the `resource_name_prefix` you put in `operations/deploy/azurebooks/subt/main.tf`
- Verify you see your resources created (like Virtual Machines, networking, etc) with your prefix.


## 3. Virtual Machine Prerequisites

**Tutorial at:** [`docs/azure-prereq.md`](azure-prereq.md)

- Follow this tutorial for setting up the prerequisites needed for using the Azure VMs.

## 4. Prepare The Virtual Machines

**Tutorial at:** [`docs/azure-prepare.md`](azure-prepare.md)

- This tutorial will install all the linux system dependencies on the Azure VMs.
- This tutorial will install the deploy repo on the VM. So, to say it again, the user should not need to clone the deploy repo or any repo dependencies on the VM manually, the `ansible` install scripts should install everything.

If there are any dependencies missing, please notify the maintainer.

## 5. Setup Docker On The Cloud Virtual Machines

This Azure **docker setup tutorial** will setup the following:

- Install all docker images -- *this sets up the repository package dependencies*
- Create all docker containers -- *the container is the environment where you will be building & running the code*

**Tutorials at:**

  - **Basestation:** [`docs/azure-docker-basestation-setup.md`](azure-docker-basestation-setup.md)
  - **UGV:** [`docs/azure-docker-ugv-setup.md`](azure-docker-ugv-setup.md)
  - **UAV:** [`docs/azure-docker-uav-setup.md`](azure-docker-uav-setup.md)

## 6. Summary

You should now have an Azure infrastructure, corresponding docker images and docker containers setup on the Azure VMs.

- Notify the maintainer if any of the tutorial steps did not succeed.

Go back to the top level [`Readme.md`](../README.md) to continue to build the catkin workspace on the Azure VMs.
