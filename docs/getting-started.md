# SubT Getting Started

## 1. Bitbucket SSH Keys

**Create SSH keys on localhost**

- **Step 1:** Generate ssh keys for subt bitbucket:

        mkdir -p ~/.ssh/
        cd ~/.ssh/
        ssh-keygen

    - Answer the prompts from `ssh-keygen` as shown below:

            Enter file in which to save the key (/home/<USER-NAME>/.ssh/id_rsa): /home/<USER-NAME>/.ssh/bitbucket
            Enter passphrase (empty for no passphrase):

    - **DO NOT ENTER A PASSPHRASE on `ssh-keygen`! LEAVE IT BLANK.**
    - **Docker will not build successfully if you have a passphrase.**
    - Replace `<USER-NAME>` with your actual username

- **Step 2:** Add the ssh bitbucket key to your localhost ssh config file:

        # create (if not created) ssh config file
        touch ~/.ssh/config

        # open the ssh config file
        gedit ~/.ssh/config

        # Add the following to the top of the config file:
        IdentityFile ~/.ssh/bitbucket

        # exit the ssh config file

**Update On Bitbucket Server**

- **Step 1:** Open a Terminal:

        # copy this entire string
        cat /home/<USER-NAME>/.ssh/bitbucket.pub

- **Step 2:** Open a Browser:

        On Bitbucket, choose Personal gear settings icon from in the lower left corner.

        Click SSH keys.

        From Bitbucket, click Add key.

        Enter a Label for your new key.

        Paste the copied public key into the SSH Key field (*from Step 1*).

        Click Save.

        Bitbucket sends you an email to confirm the addition of the key.

        Return to the command line and verify your configuration and username by entering the following command:

- **Step 3:** Refresh your ssh key connnections (now that you have added it to bitbucket)

        ssh -T git@bitbucket.org

- For more references, see [**Bitbucket instructions**](https://confluence.atlassian.com/bitbucket/set-up-an-ssh-key-728138079.html#SetupanSSHkey-Step4.AddthepublickeytoyourBitbucketsettings).

## 2. Clone the deploy repo

        mkdir ~/deploy_ws/
        cd ~/deploy_ws/
        git clone git@bitbucket.org:castacks/dsta_deploy.git src
        cd src

        # temporary for now. please make sure you are on this branch
        git checkout kat/operations-cleanup-final

## 3. Install Operations

        cd ~/deploy_ws/src

        # run the deployer operations install
        # - you only need to run this command when there are changes to operations
        # - the maintainer will notify everyone when to re-run this command
        ./install-deployer.bash --install --thirdparty

        # source your bashrc (or zsh or whichever shell you are using)
        # - or open up a new terminal
        source ~/.bashrc

## 4. Customize Configurations

Operations installs customizable configuration files in: `~/.subt/`

**General customize configurations**

Open: `~/.subt/user_config.bash`

- **Step 1**
  - If you have an nvidia card, change from `USE_NVIDIA_DRIVER=true` to `USE_NVIDIA_DRIVER=false`.

- **Step 2**
  - If you have permissions to install the SLAM modules, change `ENABLE_SLAM=false` to `ENABLE_SLAM=true`

**Ansible customize configurations**

Open: `~/.subt/ansible_config.yaml`

- **Step 1**
  - If you want to install ros on localhost, change from `install_ros=false` to `install_ros=true`.

- **Step 2**
  - If you want to install ros on localhost, change from `install_teamviewer=false` to `install_teamviewer=true`.

## 5. Install System Libraries (ansible)

        # Run the ansible install on localhost, installs all thirdparty libraries for running subt on localhost and azure
        # - the install will ask for your laptop's password
        subt cloud ansible localhost install-localhost.yaml -p

The script will fail the first time you run. It will show:

        TASK [test docker with 'hello world' example] ********************************************************************************************************************************************************************************************************************************************************************
        fatal: [localhost]: FAILED! => {"changed": true, "cmd": "docker run --rm hello-world", "delta": "0:00:00.030621", "end": "2021-03-30 21:22:05.019196", "msg": "non-zero return code", "rc": 126, "start": "2021-03-30 21:22:04.988575", "stderr": "docker: Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: Post http://%2Fvar%2Frun%2Fdocker.sock/v1.24/containers/create: dial unix /var/run/docker.sock: connect: permission denied.\nSee 'docker run --help'.", "stderr_lines": ["docker: Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: Post http://%2Fvar%2Frun%2Fdocker.sock/v1.24/containers/create: dial unix /var/run/docker.sock: connect: permission denied.", "See 'docker run --help'."], "stdout": "", "stdout_lines": []}

To solve: please reboot your computer and re-run the ansible command again:

        # Run the ansible install on localhost, installs all thirdparty libraries for running subt on localhost and azure
        # - the install will ask for your laptop's password
        subt cloud ansible localhost install-localhost.yaml -p

- You can run the ansible command from anywhere.
- Sometimes the ansible will show "errors" but will continue to run. Its OK to ignore these.
- If there are any other errors and the script crashes, then **please notify the maintainer**. See **Common Ansible Install Errors** below to solve some common errors.

## 6. Verify Installations

Verify you have all the operations tools installed correctly:

        # verify docker
        docker --version

        # verify docker-compose
        docker-compose -v

        # (optional) verify nvidia-docker
        nvidia-docker -v

        # verify ansible configuration management tools
        ansible --version

        # verify terraform cloud provisioning tool
        terraform --version

        # verify azure cli
        az --help

        # verify azcopy
        azcopy -v

        # (optional) teamviewer client for remote VM desktop access
        teamviewer --help

        # verify docker-compose shows the help usage message
        docker-compose-wrapper --help

        # verify deployer script shows the help usage message
        deployer --help

        # verify SubT autocompleter
        subt [TAB]

- Notify the maintainer if any of the `help` usage messages do not show up.

* * *

## Common Ansible Install Errors (optional):

**Error: apt-get failed**

- Check `apt-get update` shows NO ERRORS. If so, fix your `apt` sources, before re-running the ansible script.

**Error: permission denied**

- *Solution:*

    - Double check the password you entered. Make sure its the same password you use to login to your user.

- *Solution:*

        sudo visudo

        # Allow user to perform sudo on certain commands, where 'katarina' is my username
        YOUR-USER-NAME ALL=NOPASSWD: ALL

    - This is a security risk, **please remember** to remove once done with the ansible script.

**Error: ansible fails on `docker hello world`**

- Docker still needs to use `sudo` (it should work without `sudo`) because the docker group did not correctly reset in the ansible script.

- *Solution:*

        # reset your user group
        Log out and back in (or restart your computer)

        # retry & confirm docker runs without `sudo`
        docker ps

        # re-run the ansible script
        subt cloud ansible localhost install-localhost.yaml -p

**Notify the maintainer if any of the above deploy setup steps failed.**
