# UGV Launch

## 1. Azure Access

### Teamviewer

```text
# ssh into the remote VM. Example:
ssh azure.ugv1

# view teamviewer info
sudo teamviewer info

# get the teamviewer ID
sudo teamviewer info | grep "TeamViewer ID"
```

Copy the remote `TeamViewer ID` into your localhost teamviewer "Control Remote Computer" Partner ID.

- *teamviewer password:* `teamviewer`

Once in the remote TeamViewer Window, access the `subt` user's desktop

- Azure VM user `subt` password is: `Password1234!`

### RDP

If you do not prefer to use Teamveiwer, you can use RDP instead.

```text
subt tools rdp -t azure-ugv1-window -h azure-ugv1 -u subt -p Password1234!
```

## 2. Access Docker Container

```text
# ssh into the remote Azure VM
ssh azure.ugv1

# enter the docker shell container
docker-join.bash --name ugv1-shell
```

## 3. Launch UGV Simulation

```text
# load the tmux session. Example launch `ugv1`
ROBOT=ugv1 tmuxp load ~/deploy_ws/src/subt_launch/tmux/azure/ugv.yaml

# (OPTIONAL) open a new tab/window and publish a waypoint
rostopic pub --once /way_point geometry_msgs/PointStamped '{header: {frame_id: map}, point: {x: 16, y: 0, z: 0}}'

# (OPTIONAL) Move the UGV using the Basestation Azure VM GUI.
# -- On the Basestation Azure VM (example, moving ugv1):
#       Select UGV1 on both control GUIs
#       Select waypoints on rviz
```

## 4. Verify Launch

Please verify all the launch scripts in the tmux sessions do not have any errors.

## Summary

You should now be able to control the robot movement using the buttons in the basestation.
