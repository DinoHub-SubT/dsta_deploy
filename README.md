# Deploy

About...

* * *

## Getting Started

> On a new `git clone`, always perform the below setup steps.

- **Clone the repo as such:**
        
        mkdir /home/[user]/deploy_ws
        git clone git@bitbucket.org:cmusubt/deploy.git src
        cd src

- **Setup the build script:**

        git submodule init
        git submodule update ci/ci_phase
        cd ci/ci_phase
        python setup.py install
        git clean -f -d
        cd ../..

    - if you get permission denined, during `setup.py`, then do:

            python setup.py install --user


- **Verify the builder is working:**

        ./builder --help

* * *

## Builder

 **Clone your specific workspace**

- planning workspace:
  
        ./builder -y launch/build-config/planner.yaml -s clone

- perception workspace:
  
        ./builder -y launch/build-config/perception.yaml -s clone

**Build your specific workspace**

- planning workspace:

        ./builder -y launch/build-config/planner.yaml -s build

- perception workspace:

        ./builder -y launch/build-config/perception.yaml -s build

* * *

## General Structure

**Structure of the deploy repo:**

    deploy_ws/
    
      build/
        deps
        subt
        perception
      devel/
        deps
        subt
        perception
      install/
        deps
        subt
        perception

      deploy
        ci
          - jenkins scripts & config files
          - possibly ci_versioner
        docker
          - dockerfiles & config file for craeting images, running containers
        documentation
          - how to use deploy repo
        launch
          build-config
            - yaml files for automated build of the deploy repo
            - configure to build specific workspaces
          ugv_setup/ [submodule]
            - launch scripts
        field_support
          - misc. scripts (possibly non-catkin), notes, procedures, etc.
            - example: copy scripts
          
        planning
          .catkin_tools/
            - catkin tools profile configuration (repo setup) for planning
          planning_workspace [submodule]
            - deploy rosinstall
            - deploy-deps rosinstall
          
          deps/catkin/
            - perform here:
              - ln -s ../../planning_workspace/deploy-deps.rosinstall
              - wstool update
              - catkin build      
            .catkin_tools/
              - catkin tools profile configuration (repo setup) for deps
            thirdparty1/
              - any thirdparty repo not in rosinstall
        
        perception
          .catkin_tools/
            - catkin tools profile configuration (repo setup) for perception
      
          object_detection [submodule]
      
          deps/ [optional]
            - ...