# Basestation Launch

## Basestation Catkin Workspace

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

### 3. Access Docker Container

If you are using an Azure VM, remember to remote-desktop into the VM first.

If you are not using docker containers, you may skip this step.

        # enter the docker shell container on your local laptop host or Azure VM host
        docker-join.bash --name gui-shell

### 4. Launch Basestation

        # go to the top-level deploy workspace
        cd ~/deploy_ws/src

        # load the tmux session
        ROBOT=basestation tmuxp load subt_launch/tmux/azure/basestation.yaml

### Summary

Please launch the other VMs (ugv, uav), to see the robots on the basestation RViz.
