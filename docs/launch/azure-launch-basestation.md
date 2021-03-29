# Basestation Launch

## 1. Azure Access

### Teamviewer

        # ssh into the remote VM.
        ssh azure.basestation

        # view teamviewer info
        sudo teamviewer info

        # get the teamviewer ID
        sudo teamviewer info | grep "TeamViewer ID"

Copy the remote `TeamViewer ID` into your localhost teamviewer "Control Remote Computer" Partner ID.

- *teamviewer password:* `teamviewer`

Once in the remote TeamViewer Window, access the `subt` user's desktop

- Azure VM user `subt` password is: `Password1234!`

### RDP

If you do not prefer to use Teamveiwer, you can use RDP instead.

        subt tools rdp -t azure-basestation-window -h azure-basestation -u subt -p Password1234!

## 2. Access Docker Container

        # ssh into the remote Azure VM
        ssh azure.basestation

        # enter the docker shell container
        docker-join.bash -n basestation-shell

## 3. Launch Basestation

        # load the tmux session
        ROBOT=basestation tmuxp load ~/deploy_ws/src/subt_launch/tmux/simulation/basestation.yaml

## Summary

Please launch the other VMs (ugv, uav), to see the robots on the basestation RViz.
