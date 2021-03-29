# Updating Deploy

Recall, the deploy repo maintains a **3-commit level** group of submodules:

    deploy (top level, meta-repo) [submodule]
        intermediate-repo (meta-repo) [submodule]
            module-repo  [submodule]

- Please see [`discuss-install-workspaces`](discuss-install-workspaces.md) for further details.

* * *

## Tutorial: Updating Submodule Versions

### **Example Scenario**

You want to **update** the `comms_planner` module-repo to the `common` intermediate-repo.

- You want to make the change permanent in deploy for others to clone.

#### Step 1: Update comms_planner module-repo:

The module-repo is found as the `~/deploy_ws/src/common/comms_planner` submodule.

    # Go to the `comms_planner` module-repo submodule
    cd ~/deploy_ws/src/common/comms_planner

    # Check the git status
    git status

    # You will see the submodule is at a 'detached HEAD'.
    # - By default, git clones submodules as a 'detached HEAD' state. Make sure to switch to your branch during development.
    # For example you will see the output:
    #   -> HEAD detached at 0d39f91

    # Switch to a specific branch for your development
    git checkout [comms_planner's branch name]

    # You do some development
    # ... I AM DEVELOPING!! ...

    # You are done development. Now you need to update your comms_planner, the `module-repo`
    git add .
    git commit -m "update to the comms planner"

    # push your `module-repo` (the module-repo) changes to origin
    git push origin [comms_planner's branch name]

- Notice, you have been developing on your branch in `comms_planner`, not in `detached HEAD`, just like normal development.

**You have finished the first commit level:** you have updated the module-repo (submodule) `comms_planner`.

#### Step 2: Update common intermediate-repo:

The intermediate-repo is found as the `~/deploy_ws/src/common/` submodule.

    # Go to the `common` intermediate-repo submodule
    cd ~/deploy_ws/src/common/

    # Check the git status
    git status

    # You will see the `comms_planner` submodule can be prompted to commit.
    # For example you will see the output:
    #   -> modified:   comms_planner (untracked content)

    # Update `common` (the intermediate-repo) with your newest `comms_planner` (the module-repo) changes
    git add comms_planner
    git commit -m "updating common, the intermediate-repo level, with the newest comms_planner updates"

    # push `common` (the intermediate-repo) changes to origin
    git push origin [common's branch name]

- Notice, git just updates the commit hash of `comms_planner` that `common` is tracking.
- The intermediate-repo maintains submodule commit hashes of the all the module-level repos.

**You have finished the second commit level**: you have updated the intermediate-repo (submodule) `common`.

#### Step 3: Update deploy top-level-repo:

The top-level-repo is found as the `~/deploy_ws/src/` submodule.

    # Go to the `deploy` top-level-repo submodule
    cd ~/deploy_ws/src/

    # Check the git status
    git status

    # You will see the `common` submodule can be prompted to commit.
    # For example you will see the output:
    #   -> modified:   common (untracked content)

    # Update `deploy` (the top-level-repo) with your newest `common` (the intermediate-repo) changes
    git add common
    git commit -m "updating deploy, the top-level-repo level, with the newest common updates"

    # push `deploy` (the top-level-repo) changes to origin
    git push origin [deploy's branch name]

- Notice, git just updates the commit hash of `common` that `deploy` is tracking.
- The top-level-repo maintains submodule commit hashes of the all the intermediate-level repos.

**You have finished the third commit level**: you have updated the top-level-repo (submodule) `deploy`.

* * *

## Tutorial: Adding New Submodules

You want to **add** the `world_abstraction` module-repo, to the `common` intermediate-repo.

- You want to make the change permanent in deploy for others to clone.

#### Step 1: Add world_abstraction module-repo:

You want to add the module-repo as the `~/deploy_ws/src/common/world_abstraction` submodule.

    # Go to the `common` intermediate-repo submodule
    cd ~/deploy_ws/src/common/

    # Check the git status
    git status

    # Add the `world_abstraction` module-repo to common
    git submodule add -f git@bitbucket.org:cmusubt/world_abstraction.git

    # Go to `world_abstraction` module-repo, and checkout the correct branch
    cd world_abstraction
    git checkout [world_abstraction's branch name]

- Notice, you have been checked out your branch in `world_abstraction`, not in `detached HEAD`, just like normal development.

**You have finished the first commit level:** you have added the module-repo (submodule) `world_abstraction`.

#### Step 2: Update common intermediate-repo:

The intermediate-repo is found as the `~/deploy_ws/src/common/` submodule.

    # Go to the `common` intermediate-repo submodule
    cd ~/deploy_ws/src/common/

    # Check the git status
    git status

    # You will see the `world_abstraction` submodule can be prompted to commit.
    # For example you will see the output (in green color):
    #   ->  modified:   .gitmodules
	#       new file:   world_abstraction

    # Update `common` (the intermediate-repo) with your newly added `world_abstraction` (the module-repo) submodule
    git commit -m "updating common, the intermediate-repo level, adding a new submodule world_abstraction"

    # push `common` (the intermediate-repo) changes to origin
    git push origin [common's branch name]

- Notice, git adds `world_abstraction` submodule that `common` is tracking.
- The intermediate-repo maintains submodule commit hashes of the all the module-level repos.

**You have finished the second commit level**: you have updated the intermediate-repo (submodule) `common`.

#### Step 3: Update deploy top-level-repo:

The top-level-repo is found as the `~/deploy_ws/src/` submodule.

    # Go to the `deploy` top-level-repo submodule
    cd ~/deploy_ws/src/

    # Check the git status
    git status

    # You will see the `common` submodule can be prompted to commit.
    # For example you will see the output:
    #   -> modified:   common (untracked content)

    # Update `deploy` (the top-level-repo) with your newest `common` (the intermediate-repo) changes
    git add common
    git commit -m "updating deploy, the top-level-repo level, with the newest common updates"

    # push `deploy` (the top-level-repo) changes to origin
    git push origin [deploy's branch name]

- Notice, git just updates the commit hash of `common` that `deploy` is tracking.
- The top-level-repo maintains submodule commit hashes of the all the intermediate-level repos.

**You have finished the third commit level**: you have updated the top-level-repo (submodule) `deploy`.

* * *

## Tutorial: Removing Submodules

You want to **remove** the `world_abstraction` module-repo, to the `common` intermediate-repo.

- You want to make the change permanent in deploy for others to clone.

#### Step 1: Remove world_abstraction module-repo:

You want to remove the module-repo as the `~/deploy_ws/src/common/world_abstraction` submodule.

    # Go to the `common` intermediate-repo submodule
    cd ~/deploy_ws/src/common/

    # Check the git status
    git status

    # Remove the `world_abstraction` module-repo to common
    git rm -f world_abstraction

**You have finished the first commit level:** you have removed the module-repo (submodule) `world_abstraction`.

#### Step 2: Update common intermediate-repo:

The intermediate-repo is found as the `~/deploy_ws/src/common/` submodule.

    # Go to the `common` intermediate-repo submodule
    cd ~/deploy_ws/src/common/

    # Check the git status
    git status

    # You will see the `world_abstraction` submodule can be prompted to commit.
    # For example you will see the output (in green color):
    #   ->  modified:   .gitmodules
	#       deleted:    world_abstraction

    # Update `common` (the intermediate-repo), with your removed `world_abstraction` (the module-repo) submodule
    git commit -m "updating common, the intermediate-repo level, removing submodule world_abstraction"

    # push `common` (the intermediate-repo) changes to origin
    git push origin [common's branch name]

- Notice, git removes submodule `world_abstraction` submodule that `common` is tracking.
- The intermediate-repo maintains submodule commit hashes of the all the module-level repos.

**You have finished the second commit level**: you have updated the intermediate-repo (submodule) `common`.

#### Step 3: Update deploy top-level-repo:

The top-level-repo is found as the `~/deploy_ws/src/` submodule.

    # Go to the `deploy` top-level-repo submodule
    cd ~/deploy_ws/src/

    # Check the git status
    git status

    # You will see the `common` submodule can be prompted to commit.
    # For example you will see the output:
    #   -> modified:   common (untracked content)

    # Update `deploy` (the top-level-repo) with your newest `common` (the intermediate-repo) changes
    git add common
    git commit -m "updating deploy, the top-level-repo level, with the newest common updates"

    # push `deploy` (the top-level-repo) changes to origin
    git push origin [deploy's branch name]

- Notice, git just updates the commit hash of `common` that `deploy` is tracking.
- The top-level-repo maintains submodule commit hashes of the all the intermediate-level repos.

**You have finished the third commit level**: you have updated the top-level-repo (submodule) `deploy`.

* * *

## Tutorial: Moving Submodules

There is a git command to move submodules: `git mv [old path] [new path]`

However, because we have using the **3-commit level** structure, git moving is therefore between submodules, which might lead to issues.

Instead, remove & add the submodule explicitly.

- Follow the tutorial `Removing Submodules` and then `Adding Submodules`.

* * *

## Tutorial: Synchronizing Level Submodules

When someone makes an update, you will want to pull their latest changes.

### Intermediate-Repo Synchronization

Synchronizing entire intermediate-repo example steps:

    # Go to the top-level submodule
    cd ~/deploy_ws/src/

    # Remove all the submodules (from your localhost)
    git submodule deinit -f [intermediate-repo]

    # Re-clone the submodules
    git submodule update --recursive --init [intermediate-repo]

Example, synchronizing UGV:

    # go to the deploy, top-level submodule
    cd ~/deploy_ws/src/

    # == remove all the submodules (from your localhost) ==
    git submodule deinit -f ugv

    # == re-clone all the submodules ==

    # shallow clone the ugv 'intermediate-level' submodule
    git submodule update --init ugv

    # go inside the ugv, 'intermediate-level' submodule
    cd ~/deploy_ws/src/ugv

    # (required) clone the core repositories
    git submodule update --init --recursive planning-pc/
    git submodule update --init --recursive nuc/

    # (optional) clone the hardware repositories
    git submodule update --init --recursive hardware

    # (optional) clone the slam repositories
    # -- user permission restrictions, only clone if you have permissions to do so.
    git submodule update --init --recursive slam/laser_odometry

    # check the git status (please do this step and wait until command is completed)
    git status

- Please see [`discuss-install-workspaces`](discuss-install-workspaces.md) for further details on how to update different intermediate-repo levels.

### Module-Repo Synchronization

    # Go to the intermediate-repo submodule
    cd ~/deploy_ws/src/[intermediate-repo]/

    # remove all previous submodule (from your localhost)
    git submodule deinit -f [module-repo]

    # double check there is nothing in the module-repo
    ls -all [module-repo]

    # re-clone the submodules
    git submodule update --recursive --init [module-repo]

    # you will see the output as such:
    # -> Submodule path '[module-repo]': checked out '0d39f917efe279b8c192f2b6f682fdc6655213a5'

    # double check the module-repo is cloned
    ls -all [module-repo]

- If you notice the module-repo checks out whichever commit hash the the intermediate-repo has registered. So if you need a differetn version, please checkout your module-repos' correct branch version.

* * *

## Submodule Update Best Practices

Always open new branches when updating the intermediate-level or top-level repos.

Once you have validated your branches, open up a *Pull Request* and request at all levels for a tested & peer reviewed PR merge.

You can create branches in the intermediate-level submodule, so that you can easily switch between a group of module-repos for development or testing.

* * *

## Updating Docker

Project dependencies are added to their respective `dockerfiles`.

- Add any new dependencies to any files found in `operations/docker/dockerfiles` with the extension `.dockerfile`
    - *For example:* `operations/docker/dockerfiles/ugv/services/sim.dockerfile` is the dockerfile for the running the ugv in simulation.

**Example steps:**

1. Access the system

        # Example, access the azure ugv VM
        ssh azure-ugv1

2. Add the change to the `dockerfile`

    - make sure you are on the correct system when making the change.

3. Remove all containers:

        # remove dangling images caused from any previous docker builds
        docker rmi -f $(docker images -f "dangling=true" -q)

        # view all docker container names
        docker ps -a

        # stop all running containers
        docker stop [container-name]

        # remove the container
        docker rm [container-name]

4. Exit the system, go to the local host

        exit

5. Rebuild the docker image on the remote system:

        # Example:
        # Make sure you are on the localhost.
        # Build the docker image on the remote azure system
        ./deployer -s azure.ugv1.docker.image
