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
  
`docker-machine`

  - command interface to interact with the different azure machine docker daemonss

`deployer`

  - command interface to interact with `deployerfiles` found in `operations/deploy/deploybooks/robot`

`ansible`

  - command interface to interact with `ansible playbooks` found in `operations/deploy/robotbooks/`

* * *
## Tutorial Build Example Walkthrough

Setting up the full build will take a bit of time. The setups are all very similar, so to start, choose one setup and walkthrough the process once. Once more familiar with the tools, walkthrough the entire setup.

Please choose the tutorial that matches your requirements.

### Localhost

TODO: Implementation not yet ready. Please use Azure setup for now.

Builds the `SubT` workspace on the localhost, in docker containers. All dependencies are installed in docker images and maintained in dockerfiles.

Tutorials at:

  - **Basestation:** [`docs/build-basestation-docker.md`](build-basestation-docker.md)
  - **UGV:** [`docs/build-ugv-docker.md`](build-ugv-docker.md)
  - **UAV:** [`docs/build-uav-docker.md`](build-uav-docker.md)


### Azure

Builds the `SubT` workspaces on an azure VMs. The `SubT` workspace is built on the VM, in docker containers.

Tutorials at:

  - **1. Prerequisites:** [`docs/build-azure-prereq.md`](azure-prereq.md)

  - **2. VM Preparation:** [`docs/build-azure-prepare.md`](azure-prepare.md)

  - **3. Workpace Build:** [`docs/build-azure.md`](build-azure.md)

* * *

# Summary

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
        docker-join.bash --name [container name]

        # go to the workspace path
        cd ~/deploy_ws/src/path/to/workspace

        # view the catkin profiles
        catkin profile list

        # select the catkin profile
        catkin profile set [profile name]

To learn more about the available options, please use the `--preview` or `-p` command as example:

        # preview ugv options when deploying on the remote Azure VM
        ./deployer -s azure.ugv1 -p

To learn more about the available options actually do, please use the `--verbose` or `-v` option with the `preview` option

        # preview, verbose the ugv options when deploying on the remote Azure VM
        ./deployer -s azure.ugv1 -p -v

### Common Questions

- *When to re-build docker images:*

    - When docker image does not exist on the host ( `docker images` to verify)
    
    - When  changes are made to dockerfiles (dockerfiles found in: `operations/deploy/docker/dockerfiles`)

- *When to update dockerfiles:*

    - You should make changes to dockerfiles when you want to add a new workspace thirdparty dependency.

    - If you install a dependency in the container directly, please remember to put it in the dockerfile and rebuild the docker image.

