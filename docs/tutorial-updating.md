# Updating Deploy

## Updating Module Repos

- For every update to a **module-repo submodule**, there is a minimum of 3 commits needed:

    - in the *module-repo*
    - intermediate group-repo
    - meta-repo

**Example Steps:**

1. Make sure you are on the localhost

2. Go to the *module-repo* directory

        # Example, go to a common module-repo:
        cd ~/deploy_ws/src/common/rosmon/

3. Make a change to the *module-repo* or develop.

4. Lowest level commit: Commit the change in the *module-repo*

        # Example, commit the changes in module-repo level: rosmon
        cd ~/deploy_ws/src/common/rosmon/
        git add .
        git commit -m "new changes in module-repo rosmon"
        git push origin master

5. Intermediate level commit: Commit the change in the intermediate directory: common

        # Example, commit the changes in intermediate level: common
        cd ~/deploy_ws/src/common/rosmon/
        git add .
        git commit -m "new changes in intermediate-level common: updating the rosmon module"
        git push origin master

6. Top level commit: Commit the change in the meta directory: deploy

        # Example, commit the changes in meta level: deploy
        cd ~/deploy_ws/src/
        git add .
        git commit -m "new changes in meta-level deploy: updating the common, rosmon module"
        git push origin master

## Updating Dockerfiles

Project dependencies are added to their respective `dockerfiles`.

- Add any new dependencies to any files found in `operations/deploy/docker/dockerfiles` with the extension `.dockerfile`
    - *For example:* `operations/deploy/docker/dockerfiles/ugv/services/sim.dockerfile` is the dockerfile for the running the ugv in simulation.

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
