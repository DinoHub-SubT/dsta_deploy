# Build With Azure Prepare

## About

You will need to prepare the Azure VMs by installing all the dependencies you have already setup on your localhost and more.

- There are `ansible` scripts available that automates this process.

The `ansible` scripts can be found at: `operations/ansiblebooks`

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

## 2. View Ansible Script Options

        # view the help usage message
        subt cloud ansible help

        # view available local or remote system names
        subt cloud ansible -s

        # view available ansible playbooks
        subt cloud ansible -b

## 3. Install Basestation VM Dependencies

        # Verify VM Access
        ping -c 3 azure-basestation

        # Add azure-basestation to the list of known hosts, accept when asked 'are you sure you want to continue connecting'
        ssh azure.basestation
        exit

        # == Basestation VM Install ==
        # Install basic dependencies on the remote VM (system, docker, docker tools)
        # - clones the deploy repo on the remote VM (can take 30 minutes)
        # - you do not need to clone the repo on the remote VM manually, this command will do that for you.
        subt cloud ansible azure-basestation install-azure.yaml

## 4. Install UGV VM Dependencies

        # Verify VM Access
        ping -c 3 azure-ugv1

        # Add azure-ugv1 to the list of known hosts, accept when asked 'are you sure you want to continue connecting'
        ssh azure.ugv1
        exit

        # == UGV1 VM Install ==
        # Install basic dependencies on the remote VM (system, docker, docker tools)
        # - clones the deploy repo on the remote VM (can take 30 minutes)
        # - you do not need to clone the repo on the remote VM manually, this command will do that for you.
        subt cloud ansible azure-ugv1 install-azure.yaml

Apply the above steps again for all your `UGV` VMs. Change the host from `azure-ugv1` to your available Azure VM hosts.

## 5. Install UAV VM Dependencies

        # Verify VM Access
        ping -c 3 azure-uav1

        # Add azure-uav1 to the list of known hosts, accept when asked 'are you sure you want to continue connecting'
        ssh azure.uav1
        exit

        # == UAV VM Install ==
        # Install basic dependencies on the remote VM (system, docker, docker tools)
        # - clones the deploy repo on the remote VM (can take 30 minutes)
        # - you do not need to clone the repo on the remote VM manually, this command will do that for you.
        subt cloud ansible azure-uav1 install-azure.yaml

Apply the above steps again for all your `UAV` VMs. Change the host from `azure-uav1` to your available Azure VM hosts.

## 6. Install Perception VM Dependencies

        # Verify VM Access
        ping -c 3 azure-perception1

        # Add azure-perception1 to the list of known hosts, accept when asked 'are you sure you want to continue connecting'
        ssh azure.perception1
        exit

        # == Perception VM Install ==
        # Install basic dependencies on the remote VM (system, docker, docker tools)
        # - clones the deploy repo on the remote VM (can take 30 minutes)
        # - you do not need to clone the repo on the remote VM manually, this command will do that for you.
        subt cloud ansible azure-perception1 install-azure.yaml

## 7. Verify Install

Verify everything was installed correctly on all the VMs.

Example steps below show how to verify on the basestation VM:

        # access the remote VM
        ssh azure.basestation "source ~/.dsta/subtrc.bash; subt tools verify.ops"

## 8. Remote Desktop

The SubT Azure VMs has remote desktop port enabled.

**Run the RDP client script (localhost)**

        # rdp command format:
        subt tools rdp -t [window title] -h [ VM HOST ] -u subt -p Password1234! -r [ Window Resolution ]

        # example
        subt tools rdp -t basestation -h azure-basestation -u subt -p Password1234! -r 1920x1080
