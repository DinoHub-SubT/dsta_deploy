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

* * *

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

* * *

## Tutorial Build Example Walkthrough

Please choose the tutorial that matches your requirements.

## Docker

Tutorial at: [`docs/build-local-docker.md`](build-local-docker.md)

- Builds the `SubT` workspace in docker containers.
- All dependencies are installed in docker images and maintained in dockerfiles (`operations/deploy/docker/dockerfiles`).

## Azure (optional)

Tutorial at: [`docs/build-azure.md`](build-azure.md)

- Builds the `SubT` workspace on azure VMs.

    - After VM *ssh access*, the `SubT` workspace is built on the VM localhost or in the VM docker containers.

## Localhost (optional)

Tutorial at: [`docs/build-local.md`](build-local.md)

- Builds the `SubT` workspace directly on the host.
