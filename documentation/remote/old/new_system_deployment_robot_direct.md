# New System Deployment

## Robot

### Overview

During any remote deployment, **always remember to transfer the deploy workspace** from the basestation to the remote robot.

- There are different robot & robot-computer configuration available.

    Deploy       | Available
    ------------- | -------------
    robots | r1, r2
    computers | planning-pc, nuc, xavier

#### Setup SSH Keys

**Setup Bitbucket SSH Keys**

- Generate ssh keys

        mkdir -p ~/.ssh/
        cd ~/.ssh/
        ssh-keygen

    - Please answer the prompts from `ssh-keygen` as shown below:
        
            Enter file in which to save the key (/home/<USER-NAME>/.ssh/id_rsa): /home/<USER-NAME>/.ssh/bitbucket
            Enter passphrase (empty for no passphrase):

    - **DO NOT ENTER A PASSPHRASE on `ssh-keygen`! LEAVE IT BLANK.**
    - **Docker will not build successfully if you have a passphrase.**
    - Please replace `<USER-NAME>` with your actual username

- Add the generated public key to your bitbucket user: see [**STEP 4**](https://confluence.atlassian.com/bitbucket/set-up-an-ssh-key-728138079.html#SetupanSSHkey-Step4.AddthepublickeytoyourBitbucketsettings)

**Setup Deploy SSH Keys**

- Generate ssh keys

        cd ~/.ssh/
        ssh-keygen

    - Please answer the prompts from `ssh-keygen` as shown below:
        
            Enter file in which to save the key (/home/<USER-NAME>/.ssh/id_rsa): /home/<USER-NAME>/.ssh/deploy
            Enter passphrase (empty for no passphrase):

    - **DO NOT ENTER A PASSPHRASE on `ssh-keygen`! LEAVE IT BLANK.**
    - **Deployer will not run successfully if you have a passphrase.**
    - Please replace `<USER-NAME>` with your actual username

- Add the generated public key to robot's other computers to allow the deployer to have password-less ssh access.

### Choosing the Deployment

**Follow the instructions:**

- [local deploy](#markdown-header-local-deploy)
    -  if you are deploying *directly on the robot*, not over the basestation.

- [remote deploy](#markdown-header-remote-deploy)
    - if you are deploying *over the basestation*.

