# Robot Launch Setup

**Table Of Contents**

[TOC]

## Overview

Desktop launch icons are available **on the basestation**:
    
    # start the tmux session, for corresponding robot-computer
    [robot]_[computer]_start.desktop
    
    # stop the tmux session, for corresponding robot-computer
    [robot]_[computer]_stop.desktop

- If a robot-computer icon is not available, notify the [maintainer](maintainer.md).

- Start the launches in any order, in parallel.


## Launch Setup

Launch Setup is located in two places:

  - deploy configuration yamls
  - `/launch` directory

**Deploy Configuration Yamls**

Any launch procedure can be found in its corresponding deploy configuration yaml.

  - For example: `ci/deploy/field/r1/planning-pc.yaml` under the `launch` section.

If the user does not what is being launched or source, please check these yamls.

**Launch Directory**

The repository `launch/robot_launch_scripting` contains a general set of launch files per each robot computer.

## Execute Deployer Launch

### Command Line Launch

- **Start**

        # Starts any of the `robot-computer` launches
        tmuxp load tmuxp/[robot]/[computer].start.yaml

- **Stop**

        # Stops any of the `robot-computer` launches
        tmuxp load tmuxp/[robot]/[computer].stop.yaml

- **Attach**

        # Attaches a previously detached sessions.
        tmuxp load tmuxp/[robot]/[computer].attach.yaml

- **Detach**

        # Detaches any running tmux session.
        tmuxp load tmuxp/[robot]/[computer].detach.yaml

### Launch Desktop Icons

- **Start**

  - Starts any of the `robot-computer` launches.

- **Stop**

  - Stops any of the `robot-computer` launches.

    - The stop must remove the *start* tmux session.

- **Attach**

  - Attaches a previously detached sessions.

- **Detach**

  - Detaches any running tmux session.

If any of the desktop icons fail. Notify the [maintainer](maintainer.md).