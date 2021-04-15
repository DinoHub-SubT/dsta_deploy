
# Azure Cloud Infrastructure Setup

**Please follow this tutorial only if you are planning on using the Azure Cloud resource.**

[TOC]

This tutorial will outline how to setup:
- Azure VM infrastructure.
- How to install the docker setup on the remote VMs.

You have the option of building the workspaces directly on the VM or in the docker containers.

- This tutorial will follow the docker setup.

## 1. Create the Cloud Infrastructure

Follow the [Terraform Setup](azure-terraform-setup.md) instructions, to create the Azure resources.

- Creates an example azure infrastructure with VMs, virtual networking, VPN in the `SubT` resource group.
- Gives an introduction tutorial on using the `terraform` tools for maintaining the Azure resources.
- Sets up VPN access.
- Sets up remote desktop access.

## 2. Verify the Cloud Infrastructure

**Verify** the cloud infrastructure was created. Go to the [portal.azure.com](https://portal.azure.com/#home) and search for your infrastructure setup.

- In the `SubT` resource group, search for:

      # display the azure resource name prefix used for azure resources
      echo $TF_VAR_azure_resource_name_prefix

- Verify you see your resources created (like Virtual Machines, networking, etc) with your prefix.

## 3. Prepare The Virtual Machines

Follow the [Ansible Setup](azure-prepare.md) instructions, to initialize the Azure Virtual Machines.

- This tutorial will install all the linux system dependencies on the Azure VMs.
- This tutorial will install the deploy repo on the VM. So, to say it again, the user should not need to clone the deploy repo or any repo dependencies on the VM manually, the `ansible` install scripts should install everything.

If there are any dependencies missing, please notify the maintainer.

## 4. Summary

You should now have the Azure infrastructure setup and be able to access (ssh, rdp, teamviewer) the remote Azure VMs.

- Notify the maintainer if any of the tutorial steps did not succeed.

Go back to the top level [Readme.md](../README.md) to continue to build the catkin workspace on the Azure VMs.
