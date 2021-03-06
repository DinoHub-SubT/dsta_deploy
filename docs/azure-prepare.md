# Build With Azure Prepare

## About

You will need to prepare the Azure VMs by installing all the dependencies you have already setup on your localhost and more.

- There are `ansible` scripts available that automates this process.

The `ansible` scripts can be found at: `operations/ansiblebooks`

The `ansible` scripts do not give realtime output.

- You will only see the success of the commands after it has completed.
- Some commands might take a long time to complete (as long as 40 minutes for cloning all the submodules), so wait until `ansible` returns a fail or pass status.

Please run only the ansible scripts for the virtual machines you have available.

**Things to be keep in mind:**

- If a command takes longer than 30 minutes, check your VM or VPN connection (i.e. ping the vm).
- If you see an error status for any task (and it stops the install), **please notify the maintainer**.
- You can run the `basestation`, `uav`, `perception` install steps in parallel.

**Please, run the following instructions from your localhost.**

## 1. Verify Localhost Setup: Bitbucket SSH Keys

```text
# verify you have the bitbucket keys available (Verify on bitbucket website that you have setup this ssh key for SubT)
ls ~/.ssh/bitbucket
ls ~/.ssh/bitbucket.pub
```

## 2. View Ansible Script Options

```text
# view the help usage message
subt cloud ansible help

# view available local or remote system names -- these are the ansible alias names for the remote hosts
subt cloud ansible -s

# view available ansible playbooks -- these are script that will run on the remote hosts
subt cloud ansible -b
```

## 3. Install Basestation VM Dependencies

```text
# Verify VM Access
ping -c 3 azure-basestation

# Add azure-basestation to the list of known hosts, accept when asked 'are you sure you want to continue connecting'
# - You mush ssh into the VM at least once, in order to add the ssh key to your local key-store.
ssh azure.basestation
exit

# == Basestation VM Install ==
# Install basic dependencies on the remote VM (system, docker, docker tools)
# - clones the deploy repo on the remote VM (can take 30 minutes)
# - you do not need to clone the repo on the remote VM manually, this command will do that for you.
subt cloud ansible azure-basestation install-azure.yaml
```

If you see the ansible script taking too long, check the connecting by pinging the vm:

```text
ping azure-basestation
```

Sometimes the azure VPN disconnects. To solve the issue, re-connect the VPN (it sometimes takes a few tried to get re-connected).

## 4. Install UAV VM Dependencies

```text
# Verify VM Access
# - You mush ssh into the VM at least once, in order to add the ssh key to your local key-store.
ping -c 3 azure-uav1

# Add azure-uav1 to the list of known hosts, accept when asked 'are you sure you want to continue connecting'
ssh azure.uav1
exit

# == UAV VM Install ==
# Install basic dependencies on the remote VM (system, docker, docker tools)
# - clones the deploy repo on the remote VM (can take 30 minutes)
# - you do not need to clone the repo on the remote VM manually, this command will do that for you.
subt cloud ansible azure-uav1 install-azure.yaml
```

Apply the above steps again for all your `UAV` VMs. Change the host from `azure-uav1` to your available Azure VM hosts.

If you see the ansible script taking too long, check the connecting by pinging the vm:

```text
ping azure-uav1
```

Sometimes the azure VPN disconnects. To solve the issue, re-connect the VPN (it sometimes takes a few tried to get re-connected).

## 5. Install Perception VM Dependencies

```text
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
```

If you see the ansible script taking too long, check the connecting by pinging the vm:

```text
ping azure-perception1
```

Sometimes the azure VPN disconnects. To solve the issue, re-connect the VPN (it sometimes takes a few tried to get re-connected).

## 6. Verify Install

Verify everything was installed correctly on all the VMs.

Example steps below show how to verify on the basestation VM:

```text
# access the remote VM
ssh azure.basestation "source ~/.dsta/subtrc.bash; subt tools verify.ops"
```

## 7. Remote Desktop

The SubT Azure VMs has remote desktop port enabled.

**Run the RDP client script (localhost)**

```text
# rdp command format:
subt tools rdp -t [window title] -h [ VM HOST ] -u subt -p Password1234! -r [ Window Resolution ]

# example
subt tools rdp -t basestation -h azure-basestation -u subt -p Password1234! -r 1920x1080
```
