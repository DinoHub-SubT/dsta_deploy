# Deploy

About...

* * *

## Getting Started

> On a new `git clone`, always perform the below setup steps.

- **Clone the deploy repo as such:**
        
        mkdir /home/[user]/deploy_ws
        git clone git@bitbucket.org:cmusubt/deploy.git src
        cd src

- **Setup the build script:**
        
        cd /home/[user]/deploy_ws/src
        ./install-builder.bash --install

- **Verify the builder is working:**

        ./builder --help

- ** Uninstall ci_phaser: **

        cd /home/[user]/deploy_ws/src
        ./install-builder.bash --uninstall

* * *

## Builder

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

## Docker

- ** Install docker image dependencies: **

        scp [user]@perceptron.ri.cmu.edu:/project/subt/data/docker-deploy/libcudnn7-dev_7.5.1.10-1+cuda9.0_amd64.deb /home/[user]/deploy_ws/docker


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
          builder
            - yaml files for automated build of the deploy repo
            - configure to build specific workspaces
          ugv_setup/ [submodule]
            - launch scripts
          robot_launch_scripting
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