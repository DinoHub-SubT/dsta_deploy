# UGV Launch

## UGV Catkin Workspace

### 1. Access Docker Container

If you are using an Azure VM, remember to ssh into the VM first.

If you are not using docker containers, you may skip this step.

### 1. Edit Launch Files

Edit Launch File: `ugv/sim/darpa/catkin/darpa_subt/x1_control/launch/control.launch`
        
        # Change
        #       <rosparam command="load" file="$(arg config_extras)" />
        # To:
        #       <!-- <rosparam command="load" file="$(arg config_extras)" /> -->


Edit Launch File: `ugv/sim/local_planner/launch/local_planner.launch`

        # Change
        #       <remap from="/X1/cmd_vel" to="/auton_twist"/>
        # To:
        #       <!--remap from="/X1/cmd_vel" to="/auton_twist"/-->

### 2. Launch UGV Simulation

        # enter the docker shell container on your local laptop host or Azure VM host
        docker-join.bash --name sim-shell

        # load the tmux session
        tmuxp load operations/launch/tmuxp/sim/ugv.yaml

        # open a new tab/window and publish a waypoint
        rostopic pub --once /way_point geometry_msgs/PointStamped '{header: {frame_id: map}, point: {x: 16, y: 0, z: 0}}'
