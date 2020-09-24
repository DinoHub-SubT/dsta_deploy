# Basestation Launch

## 1. Azure Access

        # ssh into the remote VM. Example:
        ssh azure.basestation

        # view teamviewer info
        sudo teamviewer info

        # get the teamviewer ID
        sudo teamviewer info | grep "TeamViewer ID"

Copy the remote `TeamViewer ID` into your localhost teamviewer "Control Remote Computer" Partner ID.

- *teamviewer password:* `teamviewer`

Once in the remote TeamViewer Window, access the `subt` user's desktop

- Azure VM user `subt` password is: `Password1234!`

If you do not prefer to use Teamveiwer, you can use RDP instead.

## 2. Access Docker Container

If you are using an Azure VM, remember to remote-desktop into the VM first.

If you are not using docker containers, you may skip this step.

        # enter the docker shell container on your local laptop host or Azure VM host
        docker-join.bash --name basestation-cpu-shell

## 3. Launch Basestation

        # load the tmux session
        ROBOT=basestation tmuxp load ~/deploy_ws/src/subt_launch/tmux/azure/basestation.yaml

## Summary

Please launch the other VMs (ugv, uav), to see the robots on the basestation RViz.
