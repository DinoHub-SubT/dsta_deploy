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

------------------

notes:
workspace: /home/mtatum/workspace/subt_new/src

 Localname                          S SCM Version (Spec)           UID  (Spec)  URI  (Spec) [http(s)://...]
 ---------                          - --- --------------           -----------  ---------------------------
 base_main_class_py                   git master  (-)              aac11c79819a git@bitbucket.org:cmusubt/base_main_class_py.git
 comms_planner                      M git kpluckter-testWithRobot  0f5e56804575 git@bitbucket.org:cmusubt/comms_planner.git
 local_planner                        git dabad-use_base_node  (-) 7b8e0d5fe25f git@bitbucket.org:cmusubt/local_planner.git
 copley_ros_driver                    git master  (-)              006465e62393 git@bitbucket.org:cmusubt/copley_ros_driver.git
 pose_controller                      git master  (-)              1ac7ea1af9e0 git@bitbucket.org:cmusubt/pose_controller.git
 map_representation_interface         git master  (-)              f3fbefa9d22e git@bitbucket.org:cmusubt/map_representation_interface.git
 velocity_controller                  git master  (-)              02afcfda6fe6 git@bitbucket.org:cmusubt/velocity_controller.git
 variant                              git master  (-)              0aeed303a355 github.com/ANYbotics/variant.git
 trajectory_library_drone             git master  (-)              c0e84c12204e git@bitbucket.org:cmusubt/trajectory_library_drone.git
 trajectory_controller                git master  (-)              7423d6a2d630 git@bitbucket.org:cmusubt/trajectory_controller.git
 tflib                                git master  (-)              c0b825f30b1b git@bitbucket.org:cmusubt/tflib.git
 state_machine                        git master  (-)              525005554fa0 git@bitbucket.org:cmusubt/state_machine.git
 rqt_multiplot_plugin                 git master  (-)              8ad4b284eb79 git@bitbucket.org:cmusubt/rqt_multiplot_plugin.git
 big_pointcloud_rviz_plugin           git master  (-)              feab3a6473d6 git@bitbucket.org:cmusubt/big_pointcloud_rviz_plugin.git
 pointcloud_pose_msgs                 git master  (-)              91747a2ff056 git@bitbucket.org:cmusubt/pointcloud_pose_msgs.git
 pointcloud_map_representation        git master  (-)              86e20cbf2647 git@bitbucket.org:cmusubt/pointcloud_map_representation.git
 pid_controller                       git master  (-)              e981c582a988 git@bitbucket.org:cmusubt/pid_controller.git
 local_planner_drone                  git master  (-)              6abb24606df8 git@bitbucket.org:cmusubt/local_planner_drone.git
 drone_interface                      git master  (-)              df279cc3efa0 git@bitbucket.org:cmusubt/drone_interface.git
 darpa_command_post                   git master  (-)              f48b0f42cdd6 git@bitbucket.org:cmusubt/darpa_command_post.git
 basestation_gui_python             M git master  (-)              52084c521a79 git@bitbucket.org:cmusubt/basestation_gui_python.git
 map_compress                         git master  (-)              6407f976836e git@bitbucket.org:cmusubt/map_compress.git
 iarc7_safety                         git master  (-)              f8db2ef692ba github.com/Pitt-RAS/iarc7_safety.git
 iarc7_msgs                           git master  (-)              f71d0e91ecdd github.com/Pitt-RAS/iarc7_msgs.git
 robot_motions_server_ros             git master  (-)              613b4ab87881 git@bitbucket.org:cmusubt/robot_motions_server_ros.git
 task_manager_config                  git master  (-)              352310a52a8c git@bitbucket.org:cmusubt/task_manager_config.git
 dfs_planner                          git master  (-)              c269f4a3a2df git@bitbucket.org:cmusubt/dfs_planner.git
 terrain_analysis                     git master  (-)              75f15854c363 git@bitbucket.org:cmusubt/terrain_analysis.git
 payload_sim                          git master  (-)              4b1a93742142 git@bitbucket.org:cmusubt/payload_sim.git
 gridmapping_msgs                     git master  (-)              28b92fc45c88 git@bitbucket.org:cmusubt/gridmapping_msgs.git
 base_main_class                      git master  (-)              81f9f28e05f6 git@bitbucket.org:castacks/base_main_class.git
 openvdb_catkin                       git master  (-)              98737e1f530f git@bitbucket.org:cmusubt/openvdb_catkin.git
 catkin_simple                        git master  (-)              0e62848b12da git@github.com/catkin/catkin_simple.git
 voxmap                               git master  (-)              b2e2b23663cb git@bitbucket.org:cmusubt/voxmap.git
 ca_control_msgs                      git master  (-)              b973e07cde90 git@bitbucket.org:cmusubt/ca_control_msgs.git
 xyzpsi_state_space                   git master  (-)              f1e68d667c80 git@bitbucket.org:cmusubt/xyzpsi_state_space.git
 planning_common                      git master  (-)              f9d08ee39afc git@bitbucket.org:cmusubt/planning_common.git
 path_follower                        git master  (-)              aab6774e9062 git@bitbucket.org:cmusubt/path_follower.git
 ompl                                 git master  (-)              0b46153105c1 git@bitbucket.org:cmusubt/ompl.git
 map_representation                   git master  (-)              16e6fbf9a78b git@bitbucket.org:cmusubt/map_representation.git
 curvature_constrained_state_spaces   git master  (-)              553da7dffcb0 git@bitbucket.org:cmusubt/curvature_constrained_state_spaces.git
 coordinate                           git master  (-)              01feaef75ad0 git@bitbucket.org:cmusubt/coordinate.git
 common                               git master  (-)              98b352addf3f git@bitbucket.org:cmusubt/common.git
 circular_curve_lib                   git master  (-)              d2fa8539148c git@bitbucket.org:cmusubt/circular_curve_lib.git
 behavior_executive                   git master  (-)              fe982e2d24e9 git@bitbucket.org:cmusubt/behavior_executive.git
 at_path_planner                      git master  (-)              773163cea055 git@bitbucket.org:cmusubt/at_path_planner.git

Also detected these repositories in the workspace, add using 'wstool scrape' or 'wstool set':

 Localname                       SCM   URI  (Spec) [http(s)://...]
 ---------                       ---   ---------------------------
 behavior_tree                   --git https://0xABAD@bitbucket.org/cmusubt/behavior_tree.git
 behavior_tree_msgs              --git git@bitbucket.org:cmusubt/behavior_tree_msgs.git
 central                         --git https://bitbucket.org/cmusubt/central.git
 communication_manager           --git git@bitbucket.org:cmusubt/communication_manager.git
 darpa_command_post_mapping      --git git@bitbucket.org:cmusubt/darpa_command_post_mapping.git
 dfs_control_panel               --git git@bitbucket.org:cmusubt/dfs_control_panel.git
 entrance_calib                  --git git@bitbucket.org:cmusubt/entrance_calib.git
 general_health_monitor          --git git@bitbucket.org:cmusubt/general_health_monitor.git
 healthmonitoringsystem          --git git@bitbucket.org:cmusubt/healthmonitoringsystem.git
 image_streamer                  --git git@bitbucket.org:cmusubt/image_streamer.git
 map_assembler                   --git 
 planning_workspace              --git git@bitbucket.org:cmusubt/planning_workspace.git
 rocker_motion_analysis_assembly --git https://0xABAD@bitbucket.org/cmusubt/rocker_motion_analysis_assembly.git
 subt_auton_common               --git https://0xABAD@bitbucket.org/cmusubt/subt_auton_common.git
 total_station_node              --git git@bitbucket.org:cmusubt/total_station_node.git