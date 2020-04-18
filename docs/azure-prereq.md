# Build With Azure Prerequisites

The Azure instructions assumes that the user has already completed the *Azure Cloud Infrastructure Setup* at [`operations/deploy/azurebooks/README.md`](../operations/deploy/azurebooks/README.md).


Please follow the below instructions on your localhost deploy repository (not on the VMs).

## 1. Setup Remote Host Alias

        # open local host file
        sudo vim /etc/hosts

        # Add the remote vm hosts.
        # The below is configured to match the Azure setup tutorial in azurebooks/subt, so please use these configurations.
        # If you have a different Azure VM setup, then you will need to change these values.
        
        10.3.1.1        azure-basestation

        10.3.1.11       azure-ugv1
        10.3.1.12       azure-ugv2
        10.3.1.13       azure-ugv3

        10.3.1.51       azure-uav1
        10.3.1.52       azure-uav2
        10.3.1.53       azure-uav3
        10.3.1.54       azure-uav4

**Verify Azure VM Ping Access**

You should be able now to ping the remote host using the above alias.

        # ping the azure ugv (please test all other available ugv VMs)
        ping azure-ugv1

        # ping the azure uav1 (please test all other available uav VMs)
        ping azure-uav1

        # ping the azure basestation
        ping azure-basestation

## 2. Setup Remote Host SSH Config

Please setup the ssh config for all available Azure VMs.

        # open local ssh config file
        sudo vim ~/.ssh/config

        # Add all the remote vm ssh configuration.
        # The below is configured to match the Azure setup tutorial in azurebooks/subt, so please use these template configurations.
        # About the configuration template:
        #       - The HostName must match what was configured in /etc/hosts 
        #       - The 'IdentityFile' is the path to the ssh key used to access the Azure VM.
        #       - The default user for all the Azure VM is `subt`
        
        Host azure.ugv1
          HostName azure-ugv1
          User subt
          IdentitiesOnly yes
          IdentityFile ~/.ssh/azure_vpn
        
        Host azure.uav1
          HostName azure-uav1
          User subt
          IdentitiesOnly yes
          IdentityFile ~/.ssh/azure_vpn

        Host azure.basestation
          HostName azure-basestation
          User subt
          IdentitiesOnly yes
          IdentityFile ~/.ssh/azure_vpn

**Verify Azure VM SSH Access**

You should be able now to ssh into the remote host using the above alias.

        # ssh into the azure ugv
        ssh azure.ugv1

        # ssh into the azure uav
        ssh azure.uav1

        # ssh into the azure basestation
        ssh azure.basestation


## 3. Remote SubT Workspace Development

You will want to setup development on the remote VM. There are a few options available:

**Option 1: Clone the repository directly on the remote host**

This option will clone the deploy repository directly on the remote VM and develop as you would on the localhost.

- The `build-tutorial` will explain how to clone the `deploy repos` on all the VMs using `ansible`. Please return to the `build-tutorial` to continue with this step.

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

        # Mount remote directory (desktop IP is found on Azure Portal VPN connections )
        sshfs [desktop username][desktop IP]:/path/to/deploy/workspace/on/locahost /vm1/mountpoint/path/

        # Setup a IDE on localhost with remote editing plugin
        # Example: https://code.visualstudio.com/docs/remote/ssh

        # Remove the remote mount on remote VM host
        sudo umount /vm1/mountpoint/path/

**Option 3: Deployer Transfer (manual rsync)**

This option will copy the deploy repo to the remote VM directory.

The transfer command does a `rsync` between the localhost and remote host deploy workspaces.

- The transfer command references the setup in the `/etc/hosts` and in the `~/.ssh/config`. Please have those setup correctly.

An example `transfer` command will have the following template format:

        # go to the deploy top level path
        cd ~/deploy_ws/src
        
        # example: transfer to remote uav1 azure vm
        ./deployer -s azure.uav1.transfer.to

        # example: transfer to the remote ugv1 azure vm
        ./deployer -s azure.ugv1.transfer.to

        # example: transfer to remote basestation azure vm
        ./deployer -s azure.basestation.transfer.to

**Recommendation**

There is no good option to choose. Remote development will be difficult to manage.

- A possible recommendation is to use Option 1 and if that becomes inconvenient then try out Option 3.

- The `transfer` command can be limited by upload range. On the initial VM setup, please use Option 1 (since the deploy repo can be large).

***Option 1:** Clone the repository directly on the remote host* 

This will require the user to manually manage the VMs, docker containers and to use git as a method of repo sharing.
Use this option if you feel comfortable with git and if you are able to easily switch between the VMs.

***Option 2:** Mount a remote filesystem using SFTP*

This method seems to be vary slow. You can try this out for experimentation.

***Option 3:** Deployer Transfer (manual rsync)*

This option will have the user develop on the localhost. The `transfer` command does a `rsync` between the localhost and remote host deploy workspaces. As well, the user only needs to use git on the localhost since the remotes are synched.

After a period of development, the user issues the `transfer` command which executes the `rsync` between the localhost and remote host deploy workspaces.

The issue are:

- The user still has to manage remote docker containers for builds and launches.

- A transfer for a small code change can be slow for the development workflow.

- The user might forget to do a `transfer` to the remote VM (can transfer to a group of VMs, not just individual).

**Some Helpful Tools For Remote Development**

- `tmux`, `byobu`
- remote desktop extensions on IDE, for [example](https://code.visualstudio.com/docs/remote/remote-overview).
- `docker machine`, `docker swarm`
