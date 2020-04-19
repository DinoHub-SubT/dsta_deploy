# Build With Azure Prepare

You will need to prepare the Azure VMs with the all the dependencies you setup on your localhost.

There are `ansible` scripts available that automates this process. 

These `ansible` scripts do not give realtime output. So you will only see the success of the commands after it has completed. Some commands might take a long time to complete (as long as 20 minutes for cloning all the submodules), so please wait until `ansible` returns a fail or pass status.

Summary:

- If a command takes longer than 30 minutes, check your VM or VPN connection.
- If you have valid VM or VPN connection, please notify the maintainer.
- If you see an error status for any task, please notify the maintainer.

- The `ansible` scripts will connect to the remote VMs and do all the installs.
- Verify you have `ssh` access to the VMs.
- You can run the `basestation`, `ugv`, `uav` install steps in parallel of each other (meaning steps 3, 4, 5 can be run in parallel of each other).

**Please, do the instructions below from your localhost laptop.**


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

        # Install basic dependencies on the remote VM
        ansible-playbook -v -i inventory/azure.ini robothost.yaml --limit azure-basestation

        # Install docker & docker-tools on the remote VM
        ansible-playbook -v -i inventory/azure.ini docker.yaml --limit azure-basestation
        ansible-playbook -v -i inventory/azure.ini docker-tools.yaml --limit azure-basestation

        # Clones the deploy repo on the remote VM
        # You do not need to clone the repo on the remote VM manually, this command will do that for you.
        ansible-playbook -v -i inventory/azure.ini git-repository.yaml --limit azure-basestation

## 4. Install UGV VM Dependencies

        # Verify VM Access
        ping -c 3 azure-ugv1

        # Install basic dependencies on the remote VM
        ansible-playbook -v -i inventory/azure.ini robothost.yaml --limit azure-ugv1

        # Install docker & docker-tools on the remote VM
        ansible-playbook -v -i inventory/azure.ini docker.yaml --limit azure-ugv1
        ansible-playbook -v -i inventory/azure.ini docker-tools.yaml --limit azure-ugv1

        # Clones the deploy repo on the remote VM (can take 30 minutes)
        # You do not need to clone the repo on the remote VM manually, this command will do that for you.
        ansible-playbook -v -i inventory/azure.ini git-repository.yaml --limit azure-ugv1

Please change the host from `azure-ugv1` to all your available Azure VM  hosts.

## 5. Install UAV VM Dependencies

        # Verify VM Access
        ping -c 3 azure-uav1

        # Install basic dependencies on the remote VM
        ansible-playbook -v -i inventory/azure.ini robothost.yaml --limit azure-uav1

        # Install docker & docker-tools on the remote VM
        ansible-playbook -v -i inventory/azure.ini docker.yaml --limit azure-uav1
        ansible-playbook -v -i inventory/azure.ini docker-tools.yaml --limit azure-uav1

        # Clones the deploy repo on the remote VM (can take 20 minutes)
        # You do not need to clone the repo on the remote VM manually, this command will do that for you.
        ansible-playbook -v -i inventory/azure.ini git-repository.yaml --limit azure-uav1

Please change the host from `azure-uav1` to all your available Azure VM hosts.

## 6. Verify Install

Please, verify everything was installed correctly on all the VMs.

Example steps below show how to verify on the basestation VM:

        # access the remote VM
        ssh azure.basestation

        # verify the deploy workspaces exists
        cd ~/deploy_ws/src

        # verify git status shows: "nothing to commit, working tree clean" and are on the correct branch
        git status

        # verify the repos exist:
        ls -all basestation/

        # verify all the tools

        # verify docker
        docker --version

        # verify docker-compose
        docker-compose -v

        # verify docker-compose shows the help usage message
        docker-compose-wrapper --help
        
        # verify deployer script shows the help usage message
        ./deployer --help

        # exit the Basestation VM
        exit