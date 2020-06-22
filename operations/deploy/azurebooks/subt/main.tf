# // /////////////////////////////////////////////////////////////////////////////
# Setup the cloud provider
# // /////////////////////////////////////////////////////////////////////////////
provider "azurerm" {
  # The "feature" block is required for AzureRM provider 2.x.
  # If you're using version 1.x, the "features" block is not allowed.
  version = "~>2.0"
  features {}

  # setup credentials
  # -- please have these variables set in your ~/.bashrc (or ~/.zshrc)
  subscription_id   = var.subscription_id
  tenant_id         = var.tenant_id

  # skip provider registration
  skip_provider_registration  = "true"
}

# // /////////////////////////////////////////////////////////////////////////////
# @brief Terraform Backend
# Note: Variables are not allowed, everything must be statically written.
# // /////////////////////////////////////////////////////////////////////////////
terraform {
  # setup the backend remote for maintaining state file storage

  backend "azurerm" {

    # existing storage account (make sure exists on azure)
    storage_account_name = "subtdeployterrastate"

    # existing storage container
    container_name       = "subtdeploy-statefile-container"

    # resource group, for the storage account
    resource_group_name  = "SubT"

    # path to the statefiles on the remote backend storage container
    # !! -- PLEASE CHANGE THE USERNAME (azure username) -- !!
    key                  = "workspaces/USERNAME/terraform.tfstate"
  }
}

# // /////////////////////////////////////////////////////////////////////////////
# Load the example modules
# // /////////////////////////////////////////////////////////////////////////////
module "example" {
  source = "./modules/"

  # // /////////////////////////////////////////////////////////////////////////////
  # Resource & Network Settings
  # // /////////////////////////////////////////////////////////////////////////////

  # use existing resource group name
  user_defined_resource_group_name  = "SubT"

  # set the resource location
  resource_location                 = "eastus"

  # name prefix to be used for all resources
  # !! -- PLEASE CHANGE THE USERNAME (azure username) -- !!
  resource_name_prefix              = "USERNAME-example"

  # tag prefix
  # !! -- PLEASE CHANGE THE USERNAME (azure username) -- !!
  tag_name_prefix                   = "tag-USERNAME-example"

  # resource VNET, address space
  vnet_address_space                = "10.3.0.0/16"

  # resource SUBNET, example subnet address space with above VNET
  subnet_address_space              = "10.3.0.0/22"

  # rsource SUBNET, gateway
  gateway_address_subnet            = "10.3.10.0/24"

  # vpn settings
  vpn_address_space                 = "10.2.0.0/24"

  # vpn ca certificate
  # !! -- PLEASE CHANGE VPN CA CERTIFCATE -- !!
  vpn_ca_cert                       = "YOUR VPN CA CERTIFICATE GOES HERE"

  # // /////////////////////////////////////////////////////////////////////////////
  # General VM Settings
  # // /////////////////////////////////////////////////////////////////////////////

  # location of local ssh key to connect to remote VM
  #   -- PLEASE DO NOT CHANGE!! KEEP THE DEFAULT PATH!!
  vm_pub_ssh_key                    = "~/.ssh/azure_vpn.pub"

  # VM disk sizes (in GB)
  # !! -- PLEASE CHANGE THE VALUE TO YOUR PREFERENCE -- !!
  basestation_disk_size             = 64
  ugv_disk_size                     = 64
  uav_disk_size                     = 64
  perception_disk_size              = 100

  # VM instance types
  # !! -- PLEASE CHANGE THE VALUE TO YOUR PREFERENCE -- !!
  ugv_vm_instance                   = "Standard_F16s_v2"
  uav_vm_instance                   = "Standard_F16s_v2"
  perception_vm_instance            = "Standard_NC6"
  # Basestation available options:
  #   - choose 'Standard_F8s_v2' to only test the GUI only for the basestation
  #   - choose 'Standard_NC6' to create a GPU VM, in order to build the perception workspace for the basestation
  basestation_cpu_vm_instance       = "Standard_F16s_v2"
  basestation_gpu_vm_instance       = "Standard_NC6"

  # VM creation enable/disance
  # !! -- PLEASE CHANGE THE VALUE TO YOUR PREFERENCE -- !!
  # Basesation: create a GPU or CPU VM
  #   - after choosing the 'basestation_vm_instance' type, please also enable/disable 'enable_basestation_gpu'
  #   variable below -- depending on what type of 'basestation_vm_instance' instance was chosen
  #   - set false:  when choosing a CPU only VM instance (such as 'Standard_F8s_v2')
  #   - set true:   when choosing a GPU VM instance (such as 'Standard_NC6')
  enable_basestation_gpu            = false

  # // /////////////////////////////////////////////////////////////////////////////
  # VM Creation -- number of VMs to create options
  # // /////////////////////////////////////////////////////////////////////////////

  # !! -- PLEASE CHANGE THE TOGGLES TO YOUR PREFERENCE -- !!
  # Recommendation:
  #   - Please keep 'basic' enabled always.
  #   - Enable 'coord' if you wish to test the full coordination simulation.
  #   - Enable 'perception' if you wish to test perception simulation.

  # About:
  #   - basic simulation:         basestation, ugv1, uav1
  #   - perception simulation:    perception
  #   - coordination simulation:  ugv2, ugv3, ..., uav2, uav3, ...
  # Comments:
  #   - ENABLING ALL: creates the entire full simulation
  #   - DISABLE ANY: will remove that set of VMs from azure
  # Usage:
  #   - Enable: 1 == true
  #   - Disable: 0 == false

  # Create all the robot VMs for basic simulation setup
  # Robot VMs: basestation, ugv1, uav1
  basic_robots_toggle                 = 1

  # Create all the robot VMs for full coordination simulation setup
  # Robot VMs: ugv2, ugv3, ..., uav2, uav3, ...
  coord_robots_toggle                 = 0

  # Create all the robot VMs for full coordination simulation setup
  # Robot VMs: ugv2, ugv3, ..., uav2, uav3, ...
  perception_robots_toggle            = 0

}
