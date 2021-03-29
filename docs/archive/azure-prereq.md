# Azure Virtual Machine Prerequisites

The Azure instructions assumes that the user has already completed the *Azure Cloud Infrastructure Setup* at [Terraform Setup](../operations/azurebooks/README.md).

Please follow the below instructions **on your localhost** (not on the VMs).

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
        10.3.1.14       azure-perception1

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

        # ping the azure perception VM
        ping azure-perception1

## 2. Setup Remote Host SSH Config

Please setup the ssh config for all available Azure VMs.

        # open local ssh config file
        vim ~/.ssh/config

        # == Add the following below to the `~/.ssh/config` file (dont add the comments )==

        # enable rsa keys to use on login
        IdentityFile ~/.ssh/bitbucket

        # Add all the remote vm ssh configuration.
        # The below is configured to match the Azure setup tutorial in azurebooks/subt, so please use these template configurations.
        # About the configuration template:
        #       - The HostName must match what was configured in /etc/hosts
        #       - The 'IdentityFile' is the path to the ssh key used to access the Azure VM (the azure tutorial sets key as the path ~/.ssh/azure_vpn)
        #       - The default user for all the Azure VM is `subt`       (the azure tutorial sets the username as `subt`)

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

        Host azure.perception1
          HostName azure-perception1
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

        # ssh into the azure perception
        ssh azure.perception1

## 3. Setup Azure Docker Context

You should need to setup the remote docker daemons connections.

        # create the basestation docker context
        docker context create azure-basestation --description "azure basestation" --docker "host=ssh://azure.basestation"

        # create the ugv docker context
        docker context create azure-ugv1 --description "azure ugv1" --docker "host=ssh://azure.ugv1"
        docker context create azure-ugv2 --description "azure ugv2" --docker "host=ssh://azure.ugv2"
        docker context create azure-ugv3 --description "azure ugv3" --docker "host=ssh://azure.ugv3"

        # create the uav docker context
        docker context create azure-uav1 --description "azure uav1" --docker "host=ssh://azure.uav1"
        docker context create azure-uav2 --description "azure uav2" --docker "host=ssh://azure.uav2"
        docker context create azure-uav3 --description "azure uav3" --docker "host=ssh://azure.uav3"
        docker context create azure-uav4 --description "azure uav4" --docker "host=ssh://azure.uav4"

        # verify the docker context setup
        docker context ls
