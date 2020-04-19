# UGV Launch

## UGV Catkin Workspace

### 1. Verify Communication Manager Connection

Please verify you have setup the azure vm communication manager connection IPs

        # open the communication manager config
        gedit deploy_ws/src/common/communication_manager/config/USER_QOS_PROFILES.xml

        # make sure the IPs are added to every 'initial_peers' tag:
        
        <element>10.3.1.1</element>
        <element>10.3.1.11</element>
        <element>10.3.1.12</element>
        <element>10.3.1.13</element>
        <element>10.3.1.51</element>
        <element>10.3.1.52</element>
        <element>10.3.1.53</element>
        <element>10.3.1.54</element>

### 2. Access Docker Container

If you are using an Azure VM, remember to remote-desktop into the VM first.

If you are not using docker containers, you may skip this step.

### 3. Edit Launch Files

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

### 4. Launch UGV Simulation

        # enter the docker shell container on your local laptop host or Azure VM host
        docker-join.bash --name sim-shell

        # load the tmux session
        tmuxp load operations/launch/tmuxp/sim/ugv.yaml

        # open a new tab/window and publish a waypoint
        rostopic pub --once /way_point geometry_msgs/PointStamped '{header: {frame_id: map}, point: {x: 16, y: 0, z: 0}}'
