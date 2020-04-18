# Build With Azure Prepare

You will need to prepare the Azure VMs with the all the dependencies you setup on your localhost.

There are `ansible` scripts available that automates this process. 

These `ansible` scripts do not give realtime output. So you will only see the success of the commands after it has completed. Some commands might take a long time to complete (as long as 20 minutes for cloning all the submodules), so please wait until `ansible` returns a fail or pass status. 
- If a command takes longer than 30 minutes, check your VM or VPN connection.
- If you have valid VM or VPN connection, please notify the maintainer.
- If you see an error status for any task, please notify the maintainer.

Assuming you have `ssh` access to the Azure VMs, please follow the instructions below to setup the VM dependencies.

- You can run the `basestation`, `ugv`, `uav` install steps in parallel of each other (meaning steps 3, 4, 5 can be run in parallel of each other).

## 1. Verify Localhost Setup: Bitbucket SSH Keys

        # verify you have the bitbucket keys available (Verify on bitbucket website that you have setup this ssh key for SubT)
        ls ~/.ssh/bitbucket
        ls ~/.ssh/bitbucket.pub


## 2. Ansible Workspace

        # go to the ansible workspace
        cd ~/deploy_ws/src/operations/deploy/robotbooks

        # verify the available hosts that are setup
        ansible-playbook -v -i inventory/azure.ini robothost.yaml --list-hosts
        

## 3. Install Basestation VM Dependencies
        
        # Verify VM Access
        ping -c 3 azure-basestation

        # Install basic dependencies
        ansible-playbook -v -i inventory/azure.ini robothost.yaml --limit azure-basestation

        # Install docker & docker-tools
        ansible-playbook -v -i inventory/azure.ini docker.yaml --limit azure-basestation
        ansible-playbook -v -i inventory/azure.ini docker-tools.yaml --limit azure-basestation

        # Clone & setup the deploy repo
        ansible-playbook -v -i inventory/azure.ini git-repository.yaml --limit azure-basestation

## 4. Install UGV VM Dependencies

        # Verify VM Access
        ping -c 3 azure-ugv1

        # Install basic dependencies
        ansible-playbook -v -i inventory/azure.ini robothost.yaml --limit azure-ugv1

        # Install docker & docker-tools
        ansible-playbook -v -i inventory/azure.ini docker.yaml --limit azure-ugv1
        ansible-playbook -v -i inventory/azure.ini docker-tools.yaml --limit azure-ugv1

        # Clone & setup the deploy repo
        ansible-playbook -v -i inventory/azure.ini git-repository.yaml --limit azure-ugv1

Please change the host from `azure-ugv1` to all your available Azure VM  hosts.

## 5. Install UAV VM Dependencies

        # Verify VM Access
        ping -c 3 azure-uav1

        # Install basic dependencies
        ansible-playbook -v -i inventory/azure.ini robothost.yaml --limit azure-uav1

        # Install docker & docker-tools
        ansible-playbook -v -i inventory/azure.ini docker.yaml --limit azure-uav1
        ansible-playbook -v -i inventory/azure.ini docker-tools.yaml --limit azure-uav1

        # Clone & setup the deploy repo
        ansible-playbook -v -i inventory/azure.ini git-repository.yaml --limit azure-uav1

Please change the host from `azure-uav1` to all your available Azure VM hosts.