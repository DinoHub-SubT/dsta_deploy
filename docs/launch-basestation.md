# Basestation Launch

## Basestation Catkin Workspace

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

        # enter the docker shell container on your local laptop host or Azure VM host
        docker-join.bash --name gui-shell

### 3. Launch Basestation

        # load the tmux session
        tmuxp load operations/launch/tmuxp/sim/basestation.yaml
