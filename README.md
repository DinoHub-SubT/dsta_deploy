# Deployer

> Deployer script automates building & launching the workspaces & building the docker images.

The deployer script reads a configuration yaml, which groups shell commands, and executes a sequence of shell commands depending on which *section* is specified.

Configuration yamls are located in `deploy/field/`. The user may view & edit any configuration yaml.

## Build Workflow

The workflow to follow will look like:

1. Update the docker images
2. Start the local docker container
3. Update all rosinstall repos on the local workstation
4. Transfer the deploy repo from the *local* workstation to a *remote* workstation
5. Start the remote docker container
6. Build (in docker) the remote workspace
7. Launch (in docker) the remote workspace

- All these steps should be pre-configured. The user needs to only run the deployer script.

## Deployer Usage Overview

- Example Usage:

    `./deployer -s basestation.build.planning`


    `./deployer -s basestation.launch.planning.start`

- Options Available:

          usage: deployer [-h] [--yaml file] [--dir dir] [--all] [--sections S [S ...]]
                          [--timeout N] [--preview] [--verbose] [--ignore-tags]

          Deployer -- shell commands read from yaml configuration files.

          optional arguments:
            -h, --help            show this help message and exit
            --yaml file, -y file  specify yaml script
            --dir dir, -d dir     specify yaml directory script
            --all, -a             run all
            --sections S [S ...], -s S [S ...]
                                  sections to run
            --timeout N, -t N     allowed time to run (-1 to ignore)
            --preview, -p         preview the sections to run
            --verbose, -v         preview the shell commands to run

### Yaml Section

> A section is a *key* in the yaml that describes the sequence of shell commands to run.

- **Configuration Yaml Template:**

        - basestation:
          - echo "hello world"
          - docker:
            - echo "docker commands to run"
          - build:
            - echo "build commands to run"
            - planning:
              - echo "build planning workspace"

- The sections in the above template are: *basestation*, *docker*, *build*, *planning*.
- Section names can be nested. Shell commands are expanded as a depth first search pattern.
    - A leaf child section will run any shell commands that it contains and any other previous shell commands that are not inside another section.
    - A parent section will run all its children sections.
- Section names can be fully specified or shortly specified.
    - The section names are pattern matched to match the full section name.
    - The user can remove any intermediate section name part, in the full section name, to invoke the pattern match.
    - Example: `basestation.build.planning`, `basestation.planning`, `basestation.build` will run the same commands.

* * *

# Getting Started

## Clone the deploy workspace

1. Clone the deploy repo:
        
    `mkdir /home/$USER/deploy_ws`


    `git clone git@bitbucket.org:cmusubt/deploy.git src`

    `cd src`

2. Install builder dependencies:

    `sudo apt-get update && sudo apt-get install python-pip -y --no-install-recommends`
    `pip install setuptools PyYAML pexpect --user`

3. Setup the build script:
        
    `cd /home/$USER/deploy_ws/src`
    `./install-builder.bash --install`

4. Verify the builder is working:

    `./deployer --help`

## Install Docker

1. Remove old versions of Docker

    `sudo apt-get remove docker docker-engine docker.io`

2. Install dependencies and keys

    `sudo apt install curl apt-transport-https ca-certificates curl software-properties-common`

3. Add the official GPG key of Docker

    `curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add - `

    `sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"`


4. Install Docker

    `sudo apt-get update && sudo apt-get install docker-ce`


5. Add your user to the docker group:

    `sudo usermod -a -G docker $USER`

    - logout-log-back-in for the changes to take effect


6. Verify your Docker installation

    * **Please** do not run with `sudo docker`. Go back to Step 5 if you still cannot run as a non-root user.


    *To verify if `docker` is installed:*

    `docker -v`

    *Try running a sample container:*

    `sudo docker run hello-world`

    - You should see the message *Hello from Docker!* confirming that your installation was successfully completed.

* * *

# Deploy To The Local Host

> Before deploying on any remote host, the user must first deploy on the their local host.

- Navigate to the deploy repo

    `cd /home/$USER/deploy_ws/src`

- Install the deploy docker images

    `./deployer -s basestation.docker.image`

    - internet access required.

- Clone all the submodules & rosinstall repos
    
    `./deployer -s basestation.clone`

- Build the *basestation* workspace
    
    `./deployer -s basestation.build`

- Launch the *basestation* workspace
    
    `./deployer -s basestation.launch`

- Verify: enter the docker container

    `docker-join`

- Verify: view the tmux sessions

    `tmux list-sessions`

    `tmux a -t [name-of-session]`

# Deploy To A Remote Host

> During any remote deployment, always remember to transfer the local deploy_ws to the remote host.

- Install docker on the remote (only needed to do once)

    - follow the above *Install Docker* instructions directly on the remote and return to the local host
    - internet access required.

- Navigate to the deploy repo on the local host

    `cd /home/$USER/deploy_ws/src`

- Transfer to the remote host

    `./deployer -s [remote-name].transfer.to`

    - to see which `[remote-name]` are available, look at the configuration files found in `deploy/field/`
    - if no remote configuration yaml is setup, notify the deploy administrator

- Install the deploy docker images on the remote
    
    `cd /home/$USER/deploy_ws/src`

    `./deployer -s [remote-name].docker.image`

- Build the *remote* workspace
    
    `./deployer -s [remote-name].build`

- Launch the *remote* workspace
    
    `./deployer -s [remote-name].launch`

- Verify: enter the docker container

    `ssh [remote-name]@[remote-host]`

    `docker-join`

- Verify: view the tmux sessions

    `tmux list-sessions`

    `tmux a -t [name-of-session]`

* * *

# Updating the Deploy Repositories

- Updating the planning, perception or any other repositories is done by updating their respective rosinstalls.
    - `perception/object_detection` is a submodule. Update via git submodule commands.

## Updating the rosinstalls

All the repositories are found in `/rosinstall`. Each rosinstall specifies a workspace.

- If the repository is does not exist, add it a rosinstall.
- If the repository is does not exist, update the rosinstall with the new *commit hash*.
    - while testing, you may specifiy a branch instead of a commit hash. However, change it eventually to the specific commit hash in order to have users update them and to force jenkins builds.

Once updated, push to origin and wait for jenkins to pass the build.

After jenkins has passed the build, deploy onto the robots.


### Who do I talk to? ###

* Katarina Cujic (kcujic@andrew.cmu.edu)
