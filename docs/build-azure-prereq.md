# Build With Azure Prerequisites

The Azure instructions assumes that the user has already completed the *Azure Cloud Infrastructure Setup* at [`operations/deploy/azurebooks/README.md`](../operations/deploy/azurebooks/README.md) or already has an azure setup available with VM ssh access.

Please follow the below instructions on your localhost deploy repository (not on the VMs).

## 1. Setup Remote Host Alias

        # open local host file
        sudo vim /etc/hosts

        # Add all the remote vm hosts. You can configure the names however you like.
        # Example: add the following to the file (please confirm your VM IPs):

        10.0.2.4        azure-ugv
        10.0.2.5        azure-uav
        10.0.2.6        azure-basestation

- You should be able now to ping the remote host using the above alias.

**Verify Azure VM Ping Access**

        # ping the azure ugv
        ping azure-ugv

        # ping the azure uav
        ping azure-uav

        # ping the azure basestation
        ping azure-basestation

## 2. Setup Remote Host SSH Config

        # open local ssh config file
        sudo vim ~/.ssh/config

        # Add all the remote vm ssh configuration.  You can configure the names however you like.
        # Example: add the following to the file:

        Host azure.ugv
          HostName azure-ugv
          User [VM USER NAME]   <-- Please enter your VM username here (without brackets)
          IdentitiesOnly yes
          IdentityFile ~/.ssh/azure_vpn

        Host azure.uav
          HostName azure-uav
          User [VM USER NAME]   <-- Please enter your VM username here (without brackets)
          IdentitiesOnly yes
          IdentityFile ~/.ssh/azure_vpn

        Host azure.basestation
          HostName azure-basestation
          User [VM USER NAME]   <-- Please enter your VM username here (without brackets)
          IdentitiesOnly yes
          IdentityFile ~/.ssh/azure_vpn

- You should be able now to ssh into the remote host using the above alias.

**Verify Azure VM SSH Access**

        # ssh into the azure ugv
        ssh azure.ugv

        # ssh into the azure uav
        ssh azure.uav

        # ssh into the azure basestation
        ssh azure-basestation

## 3. Setup the Scenario Configuration Files

The user needs to list IPs for all remote hosts in configuration files called *scenarios files*.

All scenario files can be found in: `operations/deploy/scenarios`

- `azure-[computer]`: configuration for running the deployer on the azure VM
- `desktop-[computer]`: configuration for running the deployer your localhost
- `robot-[computer]`: configuration for running the deployer your the robot deployment

*Personalize the variables*

        # Go to the scenario path
        cd operations/deploy/scenarios

        # edit the appropriate scenario
        gedit operations/deploy/scenarios/[scenario type].env

  - Change `ROS_MASTER_HOSTNAME` to your preference

  - Change `ROS_MASTER_IP` to your preference

  - Change `GPU_ENABLE_TYPE` if your VM has a gpu enabled and nvidia drivers installed.

  - Change `user` to your localhost or remote VM username
  
  - Change `host` to your localhost or remote VM host alias (as edited in /etc/hosts)

  - Change `ssh_config` to your localhost or remote ssh alias (as edited in ~/.ssh/config)

  - Change `LOCAL_DEPLOY_PATH` to the deploy repo path on your localhost

  - Change `REMOTE_DEPLOY_PATH` to the deploy repo path on your remote azure VM (even if it does not exist)

You can change more default variables not listed above or can leave it as the default value.

## 4. Transfer SubT Worksapce To Azure VM

You will want to setup development on the remote VM. There are a few options available:

**Option 1: Clone the repository directly on the remote host**

This option will clone the deploy repository directly on the remote VM and develop as you would on the localhost.

- Follow the top level [`README.md`](../README.md) to clone the deploy workspace directly on the remote Azure VM.

**Option 2: Mount a remote filesystem using SFTP**

This option will mount the deploy repo found on the localhost to a directory found on the remote VM.

- This needs to be done only once when the VM starts up and the repositories will be kept in sync.

- You will need to develop on the remote repository, not on the localhost deploy repository.

Find your desktop's IP on the Azure VPN:

- Go to Virtual network gateway
- Go to Point-to-site configuration
- See the Allocated IP addresses

        # == ssh into your VM ==
        ssh [VM username]@[private VM IP]

        # Install sshfs
        sudo apt-get install sshfs

        # Create remote mount point on localhost
        mkdir /vm1/mountpoint/path/

        # Mount remote directory
        sshfs [desktop username][desktop IP]:/path/to/deploy/workspace/on/locahost /vm1/mountpoint/path/

        # Setup a IDE on localhost with remote editing plugin
        # Example: https://code.visualstudio.com/docs/remote/ssh

        # Remove the remote mount on remote VM host
        sudo umount /vm1/mountpoint/path/

**Option 3: Deployer Transfer**

This option will copy the deploy repo to the remote VM directory.

The transfer needs to be executed anytime you wish to see your local `deploy_ws` changes on the remote VM.

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # `transfer-to` command does am rsync copy to the Azure VM
        
        # example: transfer to remove uav azure vm
        ./deployer -s azure.uav.transfer.to

        # example: transfer to remove ugv azure vm
        ./deployer -s azure.ugv.transfer.to

        # example: transfer to remove basestation azure vm
        ./deployer -s azure.basestation.transfer.to
