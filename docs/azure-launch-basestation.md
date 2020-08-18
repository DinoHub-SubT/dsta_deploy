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

## 4. Transfer To Changes (optional)

The changes, outlined in this tutorial can all be done on the localhost (so you dont need to do these changes on every robot manually).

Once changed on the localhost, you can then `transfer.to` to transfer the changes from your localhost to the remote:

        # basestation transfer.to command
        ./deployer -s azure.basestation.transfer.to

If you find the `transfer.to` is too slow or not updating files during a transfer, you can change the the `transfer.to` options in the file:

        operations/deploy/scenarios/.basestation.env

You can edit the option: `deploy_rsync_opts`

- This option tells the deployer to **exclude** files during the transfer. You may change the files that get excluded.
- **Example change:** adding `--exclude=src/.git`, will reduce the time for the transfer, but you wont see any git changes reflected on the remote.

## Summary

Please launch the other VMs (ugv, uav), to see the robots on the basestation RViz.