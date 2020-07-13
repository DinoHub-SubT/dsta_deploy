# Updating Existing Robot Deployment

**Table Of Contents**

[TOC]

## Overview

Updating an existing deployment *should always* be done from the basestation, **not directly on the robot**.

- There are different robot & robot-computer configuration available.

    Workspaces       | Computer | Robot
    ------------- | -------------| -------------
    planning | planning-pc, nuc | r1, r2
    state estimation | nuc | r1, r2
    perception | nuc, xavier | r1, r2

There are two different types of updates available:
  - **repositories**
  - **docker images**

### Update the repositories

1. Maintain a feature deploy git branch:

      - a new update should start from the **develop branch**. Then checkout a new feature branch.
      - remember to update from the develop branch for any new changes:
        - `git pull --rebase origin develop`
        - `git submodule update --init --recursive`
      - remember to deal with any merge conflicts.

2. Update the repository:

    - go to the repository directly and checkout the branches for testing or development.

3. Build one of the robots computers:

     - [planning-pc](../remote/planning-pc.md)
     - [nuc](../remote/nuc.md)
     - [xavier](../remote/xavier.md)

4. Once the above steps are completed, update the rosinstalls and the submodules:

    - [rosinstalls & submodules](../rosinstall-update.md)

5. If a new workspace is required, notify the [maintainer](../maintainer.md).

### Update the Docker Image

**Only update the docker image** if a new system dependency is needed.

1. Update planning, state estimation dockerfiles

      - [planning dockerfiles](../dockerfiles-update.md)

2. Update perception dockerfiles

      - [perception dockerfiles](../dockerfiles-update.md)
