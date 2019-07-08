
# Deployer

> Deployer, a deployment automation tool.

**Table Of Contents**

[TOC]

* * *

# Overview

This deploy repo's purpose is to maintain a working version of the subt workspaces & make robot deployment easier.

Local development, basestation & robot computers can *mostly* all run the deploy repo, in [docker](https://docs.docker.com/get-started/) containers.

The advantage of using the deploy repo & running everything in docker is:

- Fast deployment on multiple robots.
- Development version are maintained, making issues easier and faster to debug.
- Launch configurations for different robots are version control maintained.
- System dependencies are version control maintained in [dockerfiles](https://docs.docker.com/engine/reference/builder/).
- Continuous integration builds & tests integrated on lab cluster severs.

Deployer automates, on multiple robots, the following tasks:

- bulding docker images
- creating & running docker containers
- building & launching workspaces

The deployer script requires a corresponding *robot deploy configuration yaml*.

- A deploy yaml lists a sequence of shell commands for different tasks.
- Deploy yamls are located in `ci/deploy/field/`


## Workflow

**Your workflow might look like:**

1. Update the dockerfiles with new dependencies

   - build the docker images on the *local* system
3. Start the docker container on the *local* system
4. Update the rosinstalls or update the submodules on the *local* system
    - wstool update the new rosinstall updated repos in the docker container
5. Transfer the deploy repository from the *local* system to the *remote* system
6. Build the docker images on the remote system
7. Start the remote docker container
8. Build the repositories, in the docker container docker, on the remote workspace
9. Launch, in the docker container, the remote workspace

All these workflow steps should already have pre-configured deploy yamls setup.

   - The user only needs to run the deployer script tasks.

   - If no yamls are present, notify the [maintainer](#who-do-i-talk-to).


## Deploy Yamls

> Deploy yamls configure robot deployment tasks.

- **Example Usage:**

    `./deployer -s basestation.build.planning`

    - runs the `build.planning` yaml section on the basestation


    `./deployer -s basestation.launch.planning.start -p -v`

    - previews the `launch.planning.start` yaml section on the basestation


### Overview

Running the deployer may look like: `./deployer -s basestation.build.planning`

- The deployer **matches command line configuration key words to the same keyword in a yaml file**.
    
    - Key words are called *phases*

- Deployer looks for any yaml file, in `ci/deploy/field/`, that matches the keyword and stops at the first found match.

    - deploy yamls are located in `ci/deploy/field/`
    
    - Yaml filenames do not need to match any keywords. Deployer matches the phase keywords *inside* the yaml.

- **Yaml Terminology**

    Deploy       | Description
    ------------- | -------------
    Step | a single bash command
    Phase | keyword that groups sequential steps
    Section | a group of phases

    - Largely inspired by [Travis CI](https://docs.travis-ci.com/user/for-beginners/).

### Specifics

A phase is a dictionary *key* in the yaml that describes the sequence of shell commands to run.

- **Deploy Yaml Example:**

        - echo "start here"
        - basestation:
          - echo "hello world"
          - docker:
            - echo "docker commands to run"
          - build:
            - echo "build commands to run"
            - planning:
              - echo "build planning workspace"
        - echo "end here"

    Deploy       | Examples
    ------------- | -------------
    Step | `echo "start here"`, `echo "hello world"`
    Phase | *keywords at any level:* **basestation, docker, build, planning**
    Section | *keywords with phase children:* **basestation, build**

**What gets executed?**

- Execution is expanded as a depth first search pattern.
  - All steps (i.e. shell commands) on the path get executed.
- Keywords can be nested.
- A keyword **with** children, is a section.
- **Section** will run:
    - all its children: steps, sections phases.
    - all same level steps, that are not inside another section.
    - all previous levels or next steps, that are not inside another section.
- A keyword **without** children, is a phase.
- **Phase** will run:
    - all its steps
    - all same level steps, that are not inside another section.
    - all previous level or next steps, that are not inside another section.

**Example output executions**

- `./deployer -s basestation`

        echo "start here"
        echo "hello world"
        echo "docker commands to run"
        echo "build commands to run"
        echo "build planning workspace"
        echo "end here"

- `./deployer -s basestation.docker`
        
        echo "start here"
        echo "hello world"
        echo "docker commands to run"
        echo "end here"

- `./deployer -s basestation.build.planning` or `./deployer -s basestation.planning`
        
        echo "start here"
        echo "hello world"
        echo "build commands to run"
        echo "build planning workspace"
        echo "end here"

**How to match?**

- Section or phase names can be fully specified or shortly specified.
  - CLI keywords are pattern matched expanded to match the full section name.
  - The user can remove any intermediate section keyword.
    - Example: `basestation.build.planning`, `basestation.planning`, `basestation.build` will all match the same section.

[[top]](#markdown-header-deployer)

* * *

# Choosing the Deployment

Please continue with a specific deployment instructions:

- [New System Deployment](#markdown-header-new-system-deployment)

- [Updating Existing Deployment](#markdown-header-updating-existing-deployment)

Once one of the above *choosing the deployment instructions* are completed, **you are done the deployment**.

[[top]](#markdown-header-deployer)

* * *

# Getting Started with Docker

Follow the below docker install instruction only if specific deployment instructions references it.

## System Requirements

- *At minimum*, Ubuntu 16.04 installed
- Internet connection
- **If installing nvidia-docker**, Nvidia drivers already installed on your host machine.

## Install Docker

1. Remove old versions of Docker

    `sudo apt-get remove docker docker-engine docker.io`

2. Install dependencies and keys

    `sudo apt install curl apt-transport-https ca-certificates curl software-properties-common`

3. Add the official GPG key of Docker

        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add - 
        sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"


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

## Install Nvidia Docker

* **Proceed with the below instructions only if you have a NVidia GPU.**

1. Remove old version of Nvidia Docker

    `docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f`

2. Install Nvidia Docker

    `sudo apt-get purge -y nvidia-docker`

3. Setup the Nvidia Docker Repository

        curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
        distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
        curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
        sudo apt-get update

4. Install Nvidia Docker (version 2):

        sudo apt-get install -y nvidia-docker2

5. Restart the Docker daemon

        sudo service docker restart

6. Verify the installation:

    *To verify if `nvidia-docker` is installed:*

    `nvidia-docker -v`

    *Try running a sample container:*

    `docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi`

    - The docker image `nvidia/cuda` requires a recent CUDA version. If you have an earlier CUDA version, then [find a tag](https://hub.docker.com/r/nvidia/cuda/tags) with an earlier version.
        - Example: `docker pull nvidia/cuda:8.0-runtime` and then run the `docker run` command with the `nvidia/cuda:8.0-runtime` image.

    - This command should print your GPU information.

[[top]](#markdown-header-deployer)

* * *

# New System Deployment

There are different types of deployments available.

- [Local Development](#markdown-header-local-development)
- [Basestation](#markdown-header-basestation)
- [Robot](#markdown-header-robot)

Each deployment has its own catkin workspace and launch configuration preconfigured & setup.

[[top]](#markdown-header-deployer)

* * *

## Local Development

### Clone the deploy workspace

- Clone the deploy repo:
        
        # make sure to create the deploy_ws in this location
        mkdir /home/$USER/deploy_ws

        # clone the deploy repo as src
        git clone git@bitbucket.org:cmusubt/deploy.git src
        cd src

- Install deployer dependencies:

        sudo apt-get update && sudo apt-get install python-pip -y --no-install-recommends
        pip install setuptools PyYAML pexpect --user

- Setup the deploy script:
        
        # deployer script must always be called from the deploy_ws/src path
        cd /home/$USER/deploy_ws/src

        # install the deployer
        ./install-deployer.bash --install

- Verify the deploy is working:

        ./deployer --help

### Install Docker

Follow the install docker [instructions](#markdown-header-install-docker)

   - **Do not** follow the *Install Nvidia Docker*. Only follow the basic docker install instructions.

Once installed once, you do not need to install again.

### Local Development Build

- Install the subt docker images:

        # build the planning docker images locally
        ./deployer -s basestation.docker.image

        # build the perception docker images locally
        ./deployer -s xavier.docker.image -i

    - internet access required.

- Start the planning docker container:

        ./deployer -s basestation.docker.start

- Start the perception docker container:

        ./deployer -s xavier.docker.start -i

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

        ./deployer -s nuc.build -i

- Build the *perception* workspace:

        ./deployer -s xavier.build -i

### Launch

- Launch any of the corresponding workspaces:

        # launch the planning workspace
        ./deployer -s basestation.launch.start

        # launch the state estimation workspace
        ./deployer -s nuc.launch.state_est.start -i

        # launch the perception workspace
        ./deployer -s xavier.launch -i

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
        ./deployer -s xavier.docker.stop -i

[[top]](#markdown-header-deployer)

* * *

## Basestation

### Clone the deploy workspace

- Clone the deploy repo:

        # make sure to create the deploy_ws in this location        
        mkdir /home/$USER/deploy_ws

        # clone the deploy repo as src
        git clone git@bitbucket.org:cmusubt/deploy.git src
        cd src

- Install deployer dependencies:

        sudo apt-get update && sudo apt-get install python-pip -y --no-install-recommends
        pip install setuptools PyYAML pexpect --user

- Setup the deploy script:
        
        # deployer script must always be called from the deploy_ws/src path
        cd /home/$USER/deploy_ws/src

        # install the deployer
        ./install-deployer.bash --install

- Verify the deploy is working:

        ./deployer --help


### Install Docker on the Basestation

Follow the install docker [instructions](#markdown-header-install-docker)

   - **Do not** follow the *Install Nvidia Docker*. Only follow the basic docker install instructions.

Once installed once, you do not need to install again.

### Setup the Workspace

- Install the subt docker images:

        # build the docker images on the basestation
        ./deployer -s basestation.docker.image

    - internet access required.

- Start the docker container:

        ./deployer -s basestation.docker.start

- Clone all the submodules & rosinstall repos:
    
        # initialize all the repos to install
        ./deployer -s basestation.init

        # clone all the repos using wstool and submodule update
        ./deployer -s basestation.up

### Basestation Build

- Start the docker container:

        ./deployer -s basestation.docker.start

- Build the *basestation* workspace:
    
        ./deployer -s basestation.build

### Launch


- Launch the *basestation* workspace:

        ./deployer -s basestation.launch

- Verify launch started correctly:

        # enter the docker container on the robot-computer
        docker-join

        # list the available tmux sessions
        tmux list-sessions

        # enter the specified tmux session
        tmux a -t [name-of-session]

### Cleanup

- Stop the docker container when turning off the basestation:

        ./deployer -s basestation.docker.stop

[[top]](#markdown-header-deployer)

* * *

## Robot


### Overview

During any remote deployment, **always remember to transfer** the basestation deploy workspace to the remote host.

- There are different robot & robot-computer configuration available.

    Deploy       | Available
    ------------- | -------------
    robots | r1, r2
    computers | planning-pc, nuc, xavier


### Install Docker on the robot

Follow the install docker [instructions](#markdown-header-install-docker)

  - **Do not** follow the *Install Nvidia Docker*. Only follow the basic docker install instructions.

  - internet access required.

All commands below are to be done on the basestation. **DO NOT GO ON THE ROBOT to run the deployer.**

### Setup the robot workspace

- Navigate to the deploy repo **on the basestation**:

        cd /home/$USER/deploy_ws/src

- Start the docker container:

        ./deployer -s basestation.docker.start

- View available robot configurations:

        ./deployer -s [robot].[computer] -p

    - Example:
    
            # preview any robot1 computer configurations for [computer]
            ./deployer -s r1.[computer] -p

            # preview any robot2 computer configurations for [computer]
            ./deployer -s r2.[computer] -p

- Transfer the deploy repo from the basestation to the remote host:

        ./deployer -s [robot].[computer].transfer.to

    - if no remote configuration yaml is setup, notify the [maintainer](#who-do-i-talk-to)

- Robot-computer specpfic pre-build steps:

    - **nuc:**
    
            # install thirdparty xsens libraries in the container before building
            ./deployer -s [robot].nuc.rosdep.xsens

- Build the robot workspace:
    
        # this will build the workspace on the remote robot
        ./deployer -s [robot].[computer].build

### Launch the robot workspace

- Desktop launch icons available **on the basestation**:
    
        # start the tmux session, for corresponding robot-computer
        [robot]_[computer]_start.desktop
    
        # stop the tmux session, for corresponding robot-computer
        [robot]_[computer]_stop.desktop

    - If not available notify the [maintainer](#who-do-i-talk-to).

- *Optional:* manual launch if desktop icons are missing:

        # start the tmux session, for corresponding robot-computer
        ./deployer -s [robot].[computer].launch.start

    - Verify launch started correctly:

            # ssh into the remote robot-computer
            ssh [remote-name]@[remote-host]

            # enter the docker container on the robot-computer
            docker-join

            # list the available tmux sessions
            tmux list-sessions

            # enter the specified tmux session
            tmux a -t [name-of-session]

### Cleanup

- Stop the docker container when turning off the robot:

    - desktop icon:

            # stop the tmux session, for corresponding robot-computer
            [robot]_[computer]_stop.desktop

    - manually:
    
            ./deployer -s [robot].[computer].docker.stop

[[top]](#markdown-header-deployer)

* * *

# Updating Existing Deployment

Updating an existing deployment *should always* be done from the basestation, not directly on the robot.

Use the deployer from the basestation, to update the repos and transfer the changes to the robot.

Use the deployer from the basestation, to build the changes on the robot.

## Workflow

**An example workflow may look like:**

- Update the rosinstall:
    - repository does not exist, add it to an existing rosinstall.
    - repository does exist, update the rosinstall with the new *commit hash*.
- Using deployer, `wstool update` on the basestation, in the docker container.
- Using deployer, transfer the updated repos to the robot.
- Using deployer, build the changes on the robot.
- Test robot changes.
- Once updated & tested, push to origin feature branch and wait for jenkins to pass the build.
- After jenkins has passed the build, deploy onto the robots.
- After testing on the robots, create a pull request and wait for jenkins to pass the PR build.
- After jenkins has passed the PR build, merge into develop & master branches.
- Add *git tags* on the develop & master branches.

If a new workspace is required to be added, notify the [maintainer](#who-do-i-talk-to).

## Types of Updates

1. **Update the workspace repositories**

    - [Local Development](#markdown-header-local-development-update)
    - [Basestation](#markdown-header-basestation-update)
    - [Robot](#markdown-header-robot-update)
      
      
2. **Update the docker image**

     - [Planning Workspace](#markdown-header-update-planning-dockerfiles)
     - [Perception Workspace](#markdown-header-update-perception-dockerfiles)
      - only update the docker image if a new system dependency is needed to be added.

    
[[top]](#markdown-header-deployer)

* * *

## Repository Updates

- Updating the any repository is done by updating their respective rosinstalls or submodules version.

### Basestation Update

- Some of the robot updates are done with rosinstalls while others are done with submodules:

    Update Type       | Workspace
    ------------- | -------------
    rosinstall | planning, state estimation
    submodules | perception

#### Update the deploy rosinstalls

Proceed with the following instructions if your update requires changing the rosinstalls.

- Navigate to the deploy repo **on the basestation**:

        cd /home/$USER/deploy_ws/src

- Update the rosinstall:

    - edit any *rosinstall* found in `rosinstall/`

    - update the repo's `commit hash` with either a branch name or a different commit hash.

        - a `commit hash` is preferred, in order to maintain specific commit versions.

- Use `wstool` to update the repo in the deploy workspace:

    - Slower, full update:

            # start the docker container on the basestation
            ./deployer -s basestation.docker.start

            # update all the repositories on the basestation, in the docker container
            ./deployer -s basestation.up

    - Faster update:

            # start the docker container on the basestation
            ./deployer -s basestation.docker.start

            # enter the docker container on the basestation
            docker-join

            # navigate to the deploy workspace, in the docker container, to the repository to update
            cd ~/deploy_ws/src/path/to/repository

            # update the repo
            wstool up

#### Update the deploy submodules

Proceed with the following instructions if your update requires changing the perception submodules.

- Navigate to the deploy repo **on the basestation**:

        cd /home/$USER/deploy_ws/src

- Update the submodules:

        # navigate to the perception submodule
        cd perception/object_detection

        # checkout a feature branch to test
        git checkout [branch]


#### Build the workspace

Follow the instructions *from* [basestation](#markdown-header-basestation-build) build.

[[top]](#markdown-header-deployer)

* * *

### Local Development Update

#### Update the repositories

Follow the instructions from basestation update:

- [Update the rosinstalls](#markdown-header-update-the-deploy-rosinstalls)

- [Update the submodules](#markdown-header-update-the-deploy-submodules)

#### Build the workspace

Follow the instructions *from* [local development](#markdown-header-local-development-build) build.

[[top]](#markdown-header-deployer)

* * *

### Robot Update

All commands below are to be done on the basestation. **DO NOT UPDATE ON THE ROBOT.**


#### Update the repositories

Follow the instructions from basestation update:

- [Update the rosinstalls](#markdown-header-update-the-deploy-rosinstalls)

- [Update the submodules](#markdown-header-update-the-deploy-submodules)


#### Build the workspace

Follow the instructions *from* [robot](#markdown-header-setup-the-robot-workspace) setup robot workspace.

[[top]](#markdown-header-deployer)

* * *

### Issues

- `wstool update` not updating on basestation:
  
    - Make sure the docker container has the network configs setup correctly:
    
            # enter the docker container on the basestation
            docker-join

            # open network configuration files
            sudo vim vim /etc/resolv.conf

            # make sure to add the following:
            nameserver 8.8.8.8

- Basestation docker container fails to start:

    - If there are any issues starting the basestation docker container:
    
            # removes the docker container from the basestation
            ./deployer -s basestation.docker.remove

            # starts the docker container on the basestation
            ./deployer -s basestation.docker.start

    - Then make sure to do the *Issues: `wstool update` not updating* `nameserver` fix.

[[top]](#markdown-header-deployer)

* * *

## Dockerfiles Updates

Updating docker images requires changing dockerfiles.

All dockerfiles for creating docker images are located in `docker/dockerfiles/subt/local/`

Scripts to build, create, run docker images and containers are found in `docker/scripts/`

Docker container options are maintained in `docker/env/field/robot/`

- Additional default docker run commands can be found in `docker/scripts/run.bash`

[[top]](#markdown-header-deployer)

### Update Planning Dockerfiles

#### Update the dockerfiles

All planning workspace dockerfiles are found in:

        # dockerfiles for non-planning workspace dependencies
        docker/dockerfiles/subt/local/osrf/
        
        # dockerfiles for planning workspace only dependencies
        docker/dockerfiles/subt/local/subt/

Any one of those dockerfiles can be changed to add a dependency.

If you are not sure where to add a new dependency, add it anywhere in the `docker/dockerfiles/subt/local/subt/ros_0.1.dockerfile`

#### Build the Docker Images

Once added, you will need to rebuild docker images on which robot or computers require the change.

- If basestation update, build the docker images on basestation:

        # build docker image on basestation
        ./deployer -s basestation.docker.image

        - internet access required.

- If robot update, build the docker images on robot:

        # transfer basestation deploy repo to remote robot
        ./deployer -s [robot].[computer].transfer.to

        # build docker image on remote robot
        ./deployer -s [robot].[computer].docker.image

    - internet access required.

    - remember, you must always transfer any changes made to the remote system.

- please see the [updating existing deployment](#markdown-header-updating-existing-deployment) for more robot docker images updates instructions.

Once the docker image is updated, rebuild your workspace and launch.

[[top]](#markdown-header-deployer)

* * *

### Update Perception Dockerfiles

The xavier is not currently running docker image for deploy repo. However, there is a docker image available for cluster builds.

All perception workspace dockerfiles are found in:

        # dockerfiles for perception workspace dependencies
        docker/dockerfiles/subt/local/tensorflow/

Any one of those dockerfiles can be changed to add a dependency.

To add a new dependency, add it anywhere in the `docker/dockerfiles/subt/local/subt/ros_0.1-source.dockerfile`

Once added, you will need to rebuild docker images on which robot or computers require the change.

In short, **to update the docker image**:

- If local perception update, install the docker images on basestation:

        # build perception docker image locally
        ./deployer -s xavier.docker.image -i

Once the docker image is updated, rebuild your workspace and launch.

[[top]](#markdown-header-deployer)

* * *

# Robot Launch Setup

Desktop launch icons are available **on the basestation**:
    
    # start the tmux session, for corresponding robot-computer
    [robot]_[computer]_start.desktop
    
    # stop the tmux session, for corresponding robot-computer
    [robot]_[computer]_stop.desktop

- If a robot-computer icon is not available, notify the [maintainer](#who-do-i-talk-to).

## Start

Start any of the `robot-computer` configuration icons you wish to use.
    
- Start the icons in any order, in parallel.

- If a *start* does not open a tmux session, then it failed. Notify the [maintainer](#who-do-i-talk-to).

## Stop

Stop the `robot-computer`, using the stop icons.

- The stop must remove the start tmux session.

- If the corresponding start session was not removed, notify the [maintainer](#who-do-i-talk-to).

## Cleanup

Final stop: a *kill-all* icon
    
- Press the *kill-all* icon once all the *stops* have completed.
  
  - Do not press the *kill-all* icon if you have not pressed the *stop* icons for each `robot-computer` configuration previously started.
    
- This will stop the basestation launch as well.

* * *

# Who do I talk to

* Katarina Cujic (kcujic@andrew.cmu.edu)

[[top]](#markdown-header-deployer)