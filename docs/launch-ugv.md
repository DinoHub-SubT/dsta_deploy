# UGV Launch

## UGV Catkin Workspace

### 1. Azure Access

        # ssh into the remote VM. Example:
        ssh azure.ugv1

        # view teamviewer info
        sudo teamviewer info

        # get the teamviewer ID
        sudo teamviewer info | grep "TeamViewer ID"

Copy the remote `TeamViewer ID` into your localhost teamviewer "Control Remote Computer" Partner ID.

- *teamviewer password:* `teamviewer`

Once in the remote TeamViewer Window, access the `subt` user's desktop

- Azure VM user `subt` password is: `Password1234!`

If you do not prefer to use Teamveiwer, you can use RDP instead.

### 2. Verify Communication Manager Connection

Please verify you have setup the azure vm communication manager connection IPs

        # open the communication manager config
        gedit ~/deploy_ws/src/common/communication_manager/config/USER_QOS_PROFILES.xml

        # make sure the IPs are added to every 'initial_peers' tag:
        
        <element>10.3.1.1</element>
        <element>10.3.1.11</element>
        <element>10.3.1.12</element>
        <element>10.3.1.13</element>
        <element>10.3.1.51</element>
        <element>10.3.1.52</element>
        <element>10.3.1.53</element>
        <element>10.3.1.54</element>

### 3. Access Docker Container

        # ssh into the remote Azure VM (if not already logged in)
        # -- if you are not using Azure, you may skip this step.
        ssh azure.ugv1

        # enter the docker shell container (if not already joined)
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name sim-shell

### 4. Edit Launch Files

Edit Launch File: `~/deploy_ws/src/ugv/sim/darpa/catkin/darpa_subt/x1_control/launch/control.launch`
        
        # Change
        #       <rosparam command="load" file="$(arg config_extras)" />
        # To:
        #       <!-- <rosparam command="load" file="$(arg config_extras)" /> -->


Edit Launch File: `~/deploy_ws/src/ugv/sim/local_planner/launch/local_planner.launch`

        # Change
        #       <remap from="/X1/cmd_vel" to="/auton_twist"/>
        # To:
        #       <!--remap from="/X1/cmd_vel" to="/auton_twist"/-->

### 5. Add Launch File (temporary, until git merged)

        # open the communication manager config
        cd ~/deploy_ws/src/ugv/sim/behavior_executive/launch

        # create the file
        gedit behavior_executive_sim_headless.launch

        # Add the following to the file:
        <?xml version="1.0"?>
        <launch>
                <node name="behavior_tree" pkg="behavior_tree" type="behavior_tree_node.py">
                <param name="config" type="string" value="$(find behavior_executive)/config/ugv_tree_sim.tree" />
                </node>

                <node name="behavior_executive" pkg="behavior_executive" type="behavior_executive" output="screen">
                <param name="execute_target" type="double" value="40." />
                <param name="dropper_connected" type="bool" value="true" />
                <param name="comm_node_count" type="int" value="10" />
                <param name="COMMS_RANGE_TIMEOUT_SECS" type="double" value="600.0" />
                <param name="MAX_RESUME_TIMEOUT_SECS" type="double" value="1200.0" />
                <param name="STOP_THRESHOLD" type="double" value="0.01" />
                <param name="HOME_THRESHOLD" type="double" value="1.0" />
                <param name="WAYPT_THRESHOLD" type="double" value="1.0" />
                <param name="WHEEL_THRESHOLD" type="double" value="0.05" />
                <param name="MOVE_THRESHOLD" type="double" value="0.05" />
                <param name="Back_Home_Idx" type="double" value="6" />
                <param name="TAKEOFF_STATUS" type="string" value="takeoff_complete" />
                <param name="STOP_CHECK_TIMEOUT" type="double" value="0.1" />
                <param name="COMMS_STATUS_TIMEOUT_SECS" type="double" value="5.0"/>
                <param name="STATE_STAY_TIMEOUT_SECS" type="double" value="180.0"/>
                <remap from="pose" to="/integrated_to_map"/>
                <remap from="/sensor_coverage_planner_command" to="/coverage_planner_pause"/>
                </node>
        </launch>


### 5. Launch UGV Simulation

        # ssh into the remote Azure VM (if not already logged in)
        # -- if you are not using Azure, you may skip this step.
        ssh azure.ugv1

        # enter the docker shell container on your local laptop host or Azure VM host
        # -- if you are not using Docker, you may skip this step.
        docker-join.bash --name sim-shell

        # Load the tmux session. Example launch `ugv1`
        tmuxp load operations/launch/tmuxp/sim/ugv1.yaml

        # (OPTIONAL) open a new tab/window and publish a waypoint
        rostopic pub --once /way_point geometry_msgs/PointStamped '{header: {frame_id: map}, point: {x: 16, y: 0, z: 0}}'

        # (OPTIONAL) Move the UGV using the Basestation Azure VM GUI. 
        # -- On the Basestation Azure VM (example, moving ugv1):
        #       Select UGV1 on both control GUIs 
        #       Select waypoints on rviz

### 6. Verify Launch

Please verify all the launch scripts in the tmux sessions do not have any errors.

If you see any launch that failed to come up, please refer to the ugv's `operations/launch/tmuxp/sim/ugv[1-N].yaml` to see which failed and then relaunch manually.

### Summary

You should now be able to control the robot movement using the buttons in the basestation.

