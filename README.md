
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

   - If no yamls are present, notify the [maintainer](#markdown-header-who-do-i-talk-to).


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

## Local Development

### Install Docker on local development

Follow the install docker [instructions](#markdown-header-install-docker)

   - Follow the *Install Nvidia Docker* if you have a Nvidia GPU.

Once installed once, you do not need to install again.

### Deploy on the local development

Follow the [local development workstation](documentation/new_system_deployment_development_direct.md) instructions.

[[top]](#markdown-header-deployer)


## Basestation

### Install Docker on the basestation

Follow the install docker [instructions](#markdown-header-install-docker)

   - **Do not** follow the *Install Nvidia Docker*. Only follow the basic docker install instructions.

Once installed once, you do not need to install again.

### Deploy on the basestation

Follow the [basestation deployment](documentation/new_system_deployment_basestation_direct.md) instructions.

[[top]](#markdown-header-deployer)

## Robot

### Install Docker on the robot

Follow the [docker install](#markdown-header-install-docker) instructions:

  - **Do not** follow the *Install Nvidia Docker*. Only follow the basic docker install instructions.

  - internet access required.

Once installed once, you do not need to install again.

### Deploy on the robot

Follow the [robot deployment](documentation/new_system_deployment_robot_direct.md) instructions.

[[top]](#markdown-header-deployer)

* * *

# Updating Existing Deployment

Updating an existing deployment *should always* be done from the basestation, **not directly on the robot**.

## Choosing the Deployment

- **Update the workspace repositories**
    1. Maintain the deploy_ws git branch:

          - a new update should start from the *develop branch*.
          - remember to always pull from develop for any new changes.

    3. Update the rosinstalls and the submodules:
     
        - [rosinstalls](#markdown-header-update-the-deploy-rosinstalls)
        - [submodules](#markdown-header-update-the-deploy-submodules)

    4. Build one of the following the workspace:

        - [local development](documentation/new_system_deployment_development_direct.md)
        - [basestation](documentation/new_system_deployment_basestation_direct.md)
        - [robot](documentation/new_system_deployment_robot_direct.md)

    - If a new workspace is required to be added, notify the [maintainer](#markdown-header-who-do-i-talk-to).

- **Update the docker image**
    
    - only update the docker image if a new system dependency is needed.
    - [planning dockerfiles](#markdown-header-planning-dockerfiles)
    - [perception dockerfiles](#markdown-header-perception-dockerfiles)

[[top]](#markdown-header-deployer)

* * *

## Repository Updates

- Updating the any repository is done by updating their respective rosinstalls or submodules version.

    Update Type       | Workspace
    ------------- | -------------
    rosinstall | planning, state estimation
    submodules | perception

### Update the deploy rosinstalls

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

### Update the deploy submodules

- Navigate to the deploy repo **on the basestation**:

        cd /home/$USER/deploy_ws/src

- Update the submodules:

        # navigate to the perception submodule
        cd perception/object_detection
        
        # checkout a feature branch to test
        git pull origin
        git checkout [branch]

[[top]](#markdown-header-deployer)

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

### Planning Dockerfiles

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

### Perception Dockerfiles

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

- If a robot-computer icon is not available, notify the [maintainer](#markdown-header-who-do-i-talk-to).

## Start

Start any of the `robot-computer` configuration icons you wish to use.
    
- Start the icons in any order, in parallel.

- If a *start* does not open a tmux session, then it failed. Notify the [maintainer](#markdown-header-who-do-i-talk-to).

## Stop

Stop the `robot-computer`, using the stop icons.

- The stop must remove the start tmux session.

- If the corresponding start session was not removed, notify the [maintainer](#markdown-header-who-do-i-talk-to).

## Cleanup

Final stop: a *kill-all* icon
    
- Press the *kill-all* icon once all the *stops* have completed.
  
  - Do not press the *kill-all* icon if you have not pressed the *stop* icons for each `robot-computer` configuration previously started.
    
- This will stop the basestation launch as well.

* * *

# Deploy Repo Branch Structure

When you have updated the deploy repo and are ready to tag a stable commit.

The deploy repo **should pass continuous integration builds** before creating a stable tag. For an introduction to *Jenkins*, please read the [jenkins wiki](https://bitbucket.org/cmusubt/ci_jenkins/wiki/Home).

- **Deploy Branch Conventions:**

    Branch       | Description
    ------------- | -------------
    feature/hotfix branches | Feature branches for deployment development.
    develop | Feature/hotfix branches merged. Not yet always ready for a stable tag. Can break jenkins.
    master | Develop branch merged. Always add a stable tag for the PR merge. Should not break jenkins.

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
- Add a *git tag*.

## Prepare a *Stable* Git Tag

Once you have tested the feature branch on the robots & jenkins has passed the tests, it is ready to *set as stable*.

**Tag a stable commit:**

- test the feature branch on the robots
- push feature branch to origin
- feature branch passes jenkins
- create a PR into *develop*
- PR passes jenkins
- merge into *develop*
- *develop* passes jenkins
- create a PR into *master*
- PR passes jenkins
- merge into *master*
- *master* passes jenkins
- tag the new *master branch* commit: `git tag stable-[version]`

[[top]](#markdown-header-deployer)

* * *

# Who do I talk to

* Katarina Cujic (kcujic@andrew.cmu.edu)

[[top]](#markdown-header-deployer)