# Build With Azure Prepare

## About

You will need to prepare the Azure VMs by installing all the dependencies you have already setup on your localhost and more.

- There are `ansible` scripts available that automates this process.

The `ansible` scripts can be found at: `operations/deploy/robotbooks`

- Users can add to the scripts if there are missing dependencies.

The `ansible` scripts do not give realtime output.

- You will only see the success of the commands after it has completed.
- Some commands might take a long time to complete (as long as 20 minutes for cloning all the submodules), so wait until `ansible` returns a fail or pass status.

**Things to be keep in mind:**

- If a command takes longer than 30 minutes, check your VM or VPN connection.
- If you see an error status for any task (and it stops the install), **please notify the maintainer**.
- The `ansible` scripts will connect to the remote VMs and run the installs. Verify you have `ssh` access to the VMs.
- You can run the `basestation`, `ugv`, `uav` install steps in parallel of each other (meaning steps 3, 4, 5 can be run in parallel of each other).

**Please, run the following instructions from your localhost.**

## 1. Verify Localhost Setup: Bitbucket SSH Keys

        # verify you have the bitbucket keys available (Verify on bitbucket website that you have setup this ssh key for SubT)
        ls ~/.ssh/bitbucket
        ls ~/.ssh/bitbucket.pub

## 2. Install Basestation VM Dependencies

        # Verify VM Access
        ping -c 3 azure-basestation

        # Add azure-basestation to the list of known hosts, accept when asked 'are you sure you want to continue connecting'
        ssh azure.basestation
        exit

        # == Basestation VM Install ==
        # Install basic dependencies on the remote VM (system, docker, docker tools)
        # Clones the deploy repo on the remote VM (can take 30 minutes)
        # You do not need to clone the repo on the remote VM manually, this command will do that for you.
        subtani_install.sh azure-basestation install-azure.yaml

## 3. Install UGV VM Dependencies

        # Verify VM Access
        ping -c 3 azure-ugv1

        # Add azure-ugv1 to the list of known hosts, accept when asked 'are you sure you want to continue connecting'
        ssh azure.ugv1
        exit

        # == UGV1 VM Install ==
        # Install basic dependencies on the remote VM (system, docker, docker tools)
        # Clones the deploy repo on the remote VM (can take 30 minutes)
        # You do not need to clone the repo on the remote VM manually, this command will do that for you.
        subtani_install.sh azure-ugv1 install-azure.yaml

Apply the above steps again for all your `UGV` VMs. Change the host from `azure-ugv1` to your available Azure VM hosts.

## 4. Install UAV VM Dependencies

        # Verify VM Access
        ping -c 3 azure-uav1

        # Add azure-uav1 to the list of known hosts, accept when asked 'are you sure you want to continue connecting'
        ssh azure.uav1
        exit

        # == UAV VM Install ==
        # Install basic dependencies on the remote VM (system, docker, docker tools)
        # Clones the deploy repo on the remote VM (can take 30 minutes)
        # You do not need to clone the repo on the remote VM manually, this command will do that for you.
        subtani_install.sh azure-uav1 install-azure.yaml

Apply the above steps again for all your `UAV` VMs. Change the host from `azure-uav1` to your available Azure VM hosts.

## 5. Install Perception VM Dependencies

        # Verify VM Access
        ping -c 3 azure-perception1

        # Add azure-perception1 to the list of known hosts, accept when asked 'are you sure you want to continue connecting'
        ssh azure.perception1
        exit

        # == Perception VM Install ==
        # Install basic dependencies on the remote VM (system, docker, docker tools)
        # Clones the deploy repo on the remote VM (can take 30 minutes)
        # You do not need to clone the repo on the remote VM manually, this command will do that for you.
        subtani_install.sh azure-perception1 install-azure.yaml

## 6. Verify Install

Verify everything was installed correctly on all the VMs.

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

## 7. Remote Desktop

The SubT Azure VMs has remote desktop port enabled.

**Run the RDP client script (localhost)**

        # template
        azure-rdp --title [window title] --host [ VM HOST ] --user subt --pass Password1234! --res 1920x1080

        # example
        azure-rdp --title basestation --host azure-basestation --user subt --pass Password1234! --res 1920x1080
