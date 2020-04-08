# Deployment Operation Tools

[TOC]

## Prerequisites

Verify you have all the operations tools installed correctly:

        # Go to the deployer top level path
        cd ~/deploy_ws/src

        # verify docker
        docker --version

        # (optional) verify nvidia-docker
        nvidia-docker -v

        # verify docker-compose
        docker-compose -v

        # source your bashrc (or zshrc)
        source ~/.bashrc

        # verify docker-compose shows the help usage message
        docker-compose-wrapper --help
        
        # verify deployer script shows the help usage message
        ./deployer --help


## About The Deployment Operation Tools

There are a few operational tools available to use:

`docker`

  - is the command interface to interact with `dockerfiles` found in `operations/deploy/docker/dockerfiles`
  
`docker-compose`

  - is the command interface to interact with `docker-compose.yml` files found in `operations/deploy/docker/dockerfiles/`
  
`docker-compose-wrapper`

  - is the command interface to wrap `docker compose` and with `scenario` configuration files found in `operations/deploy/scenarios`
  
`deployer`

  - is the command interface to interact with `deployerfiles` found in `operations/deploy/deploybooks/robot`


* * *

# Tutorial Example Walkthrough

Please chose the tutorial that matches your requirements.

It is recommended you follow the tutorials: *Docker Deployment Example Walkthrough*, *Azure Deployment Example Walkthrough*.

**Localhost Example Walkthrough (Optional)**

- Tutorial at: [`docs/build-local.md`](build-local.md)

- Builds the `SubT` workspace directly on the host.

**Docker Deployment Example Walkthrough (Recommended)**

- Tutorial at: [`docs/build-local-docker.md`](build-local-docker.md)
- Everything will build and run in docker containers
- All dependences will be installed using docker images.

**Azure Deployment Example Walkthrough (Recommended)**

- Tutorial at: [`docs/build-azure.md`](build-azure.md)
- Everything will run in azure VMs.
- Follows either (user's choice) the local direct or local docker build tutorials.
