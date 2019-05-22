# Deploy


* * *

# Getting Started

> On a new `git clone`, always perform the below setup steps.

- **Clone the deploy repo:**
        
        mkdir /home/$USER/deploy_ws
        git clone git@bitbucket.org:cmusubt/deploy.git src
        cd src

- ** Install builder dependencies **

      sudo apt-get update && sudo apt-get install python-pip -y --no-install-recommends
      pip install setuptools PyYAML pexpect --user

- **Setup the build script:**
        
        cd /home/$USER/deploy_ws/src
        ./install-builder.bash --install
    
    * please make sure you have python & pip installed.

- **Verify the builder is working:**

        ./builder --help

* * *

# Builder

> Builder script automates building the workspaces & docker images.

## Install Builder Depends

- **Manually install on every workstation:**

      sudo apt-get update && sudo apt-get install python-pip -y --no-install-recommends
      pip install setuptools PyYAML pexpect --user

## Build Workflow


1. Install the docker images
2. Transfer the deploy repo from *main workstation* to remote workstation
3. Create the docker container
4. Clone & update the repo workspaces on the remote (in docker)
5. Build the remote (in docker)

All of these steps are setup in the *build configuration yamls* files, located in `build/`.
The user only needs to use the `builder` to select which commands to run, via selecting *tag* sections.

### Builder CLI

> builder arguments at minimum requires a yaml file & tag section.

    ./builder --help
    usage: builder [-h] [--yaml file] [--all] [--sections S [S ...]] [--timeout N]
                  [--preview] [--verbose]

    Builder phase run tool -- commands read from yaml configuration files.

    optional arguments:
      -h, --help            show this help message and exit
      --yaml file, -y file  specify yaml script
      --all, -a             run all
      --sections S [S ...], -s S [S ...]
                            sections to run
      --timeout N, -t N     allowed time to run (-1 to ignore)
      --preview, -p         run all
      --verbose, -v         run all

- Example usage:

      ./builder -y build/field/example/robot.yaml -s docker.build
      ./builder -y build/field/example/robot.yaml -s docker.stop docker.remove
      ./builder -y build/field/example/robot.yaml -s deploy.build
      ./builder -y build/field/example/transfer.yaml -s robot.to


## Field Build Example

*Based on the example yamls in:* `build/field/example/`
* Please modify to match your robot specific configuration

1. Install the docker images

        ./builder -y build/field/example/robot.yaml -s docker.build

2. Transfer the deploy repo from *main workstation* to remote workstation

        ./builder -y build/field/example/transfer.yaml -s robot.to

3. Create the docker container

        ./builder -y build/field/example/robot.yaml -s docker.start

4. Clone & update the repo workspaces on the remote (in docker)

        ./builder -y build/field/example/robot.yaml -s deploy.clone

5. Build the remote (in docker)

        ./builder -y build/field/example/robot.yaml -s deploy.build


## Local Build Example

**Clone your specific workspace**

- planning workspace:
  
        ./builder -y build/local/planning.yaml -s clone

- perception workspace:
  
        ./builder -y build/local/perception.yaml -s clone


**Build your specific workspace**

- planning workspace:

        ./builder -y build/local/planning.yaml -s build

- perception workspace:

        ./builder -y build/local/perception.yaml -s build

* * *

# General Structure

**Structure of the deploy repo:**

    
    deploy_ws/  # top level directory

      #############################################################################################
      # catkin build, devel, install
      #############################################################################################
      build/
        subt
          deps
          repo
        perception
          deps
          repo
      devel/
        subt
          deps
          repo
        perception
          deps
          repo
      install/
        subt
          deps
          repo
        perception
          deps
          repo

      #############################################################################################
      # deploy source
      #############################################################################################
      deploy
        build
          - yaml files for automating the build & setup of the deploy repo
        ci
          - jenkins scripts & config files
        docker
          - dockerfiles for each repo workspace
          - configuration files for creating, running, removing containers & images
        launch
          ugv_setup/ [submodule]
            - launch scripts
          robot_launch_scripting
            - launch scripts
        field_support
          - misc. scripts (possibly non-catkin), notes, procedures, etc.
          
        ############################################################################################
        # repo workspaces
        ############################################################################################
        # planning workspace: see submodule for details
        planning
          .catkin_tools/
            - catkin tools profile configuration (repo setup) for planning
          planning_workspace [submodule]
            - deploy.rosinstall
            - deploy-deps.rosinstall
          # planning workspace dependencies source (those not found in the rosinstalls & requires source build)
          deps/catkin/
            .catkin_tools/
              - catkin tools profile configuration (repo setup) for deps
            thirdparty1/
              - any thirdparty repo that requires source build
              - added in deploy-deps.rosinstall, cloned here.
            ...
        # perception workspace: see submodule for details
        perception
          .catkin_tools/
            - catkin tools profile configuration (repo setup) for perception
          object_detection [submodule]
          deps/catkin/ [optional]
            - ...

* * *

# Maintenance

> Two updates: one to the actual repo and one to the deploy repos' submodule git hash reference

- Summary:
    - The repo source workspaces are setup as submodules.
    - Any changes made in the actual repos will not affect the deploy repo until the submodule is updated.
    - Update the submodule to point to the git commit hash found in the newly changed actual repo.
    - Then update the deploy repo with the submodule change.

* * *

# Continuous Integration

- Jenkins is setup to run a build on each git commit change to the deploy repo.
- Two types of jobs currently exist: planning & perception workspace builds.
- See the status (checkmarks or exclamation mark) on the repo commits pages.
- More job types will be added.