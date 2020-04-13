# Build Deployment Operation Tools

[TOC]

## About Operation Tools

There are a few operational tools available to use:

`docker`

  - command interface to interact with `dockerfiles` found in `operations/deploy/docker/dockerfiles`
  
`docker-compose`

  - command interface to interact with `docker-compose.yml` files found in `operations/deploy/docker/dockerfiles/`
  
`docker-compose-wrapper`

  - command interface to wrap `docker compose` and with `scenario` configuration files found in `operations/deploy/scenarios`
  
`deployer`

  - command interface to interact with `deployerfiles` found in `operations/deploy/deploybooks/robot`


## Tutorial Build Example Walkthrough

Please choose the tutorial that matches your requirements.

### Docker

Builds the `SubT` workspace on the localhost, in docker containers.

- All dependencies are installed in docker images and maintained in dockerfiles (`operations/deploy/docker/dockerfiles`).

**Tutorials at:**

  - Basestation: [`docs/build-basestation-docker.md`](build-basestation-docker.md)
  - UGV: [`docs/build-ugv-docker.md`](build-ugv-docker.md)
  - UAV: [`docs/build-uav-docker.md`](build-uav-docker.md)


### Azure (optional)

Builds the `SubT` workspaces on an azure VMs.

- After VM *ssh access*, the `SubT` workspace is built on the VM, in docker containers.

**Tutorial Prerequisites:** [`docs/build-azure-prereq.md`](build-azure-prereq.md)

**Tutorial at:** [`docs/build-azure.md`](build-azure.md)

## Summary

You should now have a built `SubT` workspace.

You should become more familiar with operational tools for different types of operations:

**Template: Creating Docker Images**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # Create the docker image
        ./deployer -s [deployment host].[(optional) robot].[(optional) computer].docker.image

**Template: Creating Docker Shell Access Containers**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # Create the docker shell container
        ./deployer -s [deployment host].[(optional) robot].[(optional) computer].docker.shell

**Template: Building the catkin workspace**

        # enter the docker shell container
        docker-join.bash --name gui-shell

        # go to the workspace path
        cd ~/deploy_ws/src/path/to/workspace

        # view the catkin profiles
        catkin profile list

        # select the catkin profile
        catkin profile set [profile name]

To learn more about the available options, please use the `--preview` or `-p` command as example:

        # preview ugv options when deploying on the dekstop
        ./deployer -s desktop.ugv -p

To learn more about the available options actually do, please use the `--verbose` or `-v` option with the `preview` option

        # preview, verbose the ugv options when deploying on the dekstop
        ./deployer -s desktop.ugv -p -v


### Common Questions

- *How to view the docker images built on the localhost:*

        docker images

- *When to re-build docker images:*

    - When you do not have the docker image on your localhost
    
    - When  changes are made to dockerfiles (dockerfiles found in: `operations/deploy/docker/dockerfiles`)

- *When to update dockerfiles:*

    - You should make changes to dockerfiles when you want to add a new workspace thirdparty dependency.

    - If you install a dependency in the container directly, please remember to put it in the dockerfile and rebuild the docker image.
