# New System Deployment

## Local Development

### Overview

Local development involves running the planning or perception workspaces.

There are different docker images to run different workspaces.

### Local Deploy

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

#### Clone the deploy workspace

- Clone the deploy repo:
        
        # make sure to create the deploy_ws in this location
        mkdir /home/$USER/deploy_ws

        # clone the deploy repo as src
        git clone git@bitbucket.org:cmusubt/deploy.git src
        cd src

- Install deployer dependencies:

        sudo apt-get update && sudo apt-get install python-pip -y --no--localnstall-recommends
        pip install setuptools PyYAML pexpect --user

- Setup the deploy script:
        
        # deployer script must always be called from the deploy_ws/src path
        cd /home/$USER/deploy_ws/src

        # install the deployer
        ./install-deployer.bash ---localnstall

        # source your bash or zsh
        source ~/.bashrc
        source ~/.zshrc

- Verify the deploy is working:

        ./deployer --help

### Docker Build

- Install the subt docker images:

        # build the planning docker images locally
        ./deployer -s basestation.docker.image

        # build the perception docker images locally
        ./deployer -s xavier.docker.image --local

    - internet access required.

- Start the planning docker container:

        ./deployer -s basestation.docker.start

- Start the perception docker container:

        ./deployer -s xavier.docker.start --local

- Clone all the workspaces:
    
        # initialize all the planning repos to install
        ./deployer -s basestation.init

        # clone all the repos using wstool and submodule update
        ./deployer -s basestation.up

### Build

- Start the docker container:

        ./deployer -s basestation.docker.start

- Build the *planning* workspace:
    
        ./deployer -s basestation.build

- Build the *state estimation* workspace:

        ./deployer -s nuc.build --local

- Build the *perception* workspace:

        ./deployer -s xavier.build --local

### Launch

- Launch any of the corresponding workspaces:

        # launch the planning workspace
        ./deployer -s basestation.launch.start

        # launch the state estimation workspace
        ./deployer -s nuc.launch.state_est.start --local

        # launch the perception workspace
        ./deployer -s xavier.launch --local

- Verify launch started correctly:

        # enter the docker container on the robot-computer
        docker-join

        # list the available tmux sessions
        tmux list-sessions

        # enter the specified tmux session
        tmux a -t [name-of-session]

### Cleanup

- Stop the docker container when turning off the local workstation:

        # stop the planning, state estimation docker container
        ./deployer -s basestation.docker.stop

        # stop the perception docker container
        ./deployer -s xavier.docker.stop --local
