# Basestation Launch

## 1. Access Docker Container

```text
# enter the docker shell container (if not already joined)
docker-join.bash -n basestation-shell
```

## 2. Launch Basestation

```text
# load the tmux session
ROBOT=basestation tmuxp load ~/deploy_ws/src/subt_launch/tmux/localhost/basestation.yaml
```

## Summary

Please launch the other VMs (ugv, uav), to see the robots on the basestation RViz.
