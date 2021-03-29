# Robot UGV Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the ugv robots.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **basestation**.

## 1. Docker Images

**Pull Docker Images from Azure**

Follow these steps, **on the basestation**.

        # azure registry login
        az acr login --name subtexplore

        # (ROBTS HAVE INTERNET) pull the docker images from the azure docker registry

        subt deployer robots.ugv.ugv1.ppc.docker.registry.azure.pull
        subt deployer robots.ugv.ugv1.nuc.docker.registry.azure.pull
        subt deployer robots.ugv.ugv1.xavier.docker.registry.azure.pull

        # (ROBTS DO NOT HAVE INTERNET) pull the docker images from the basestation docker registry

        # step 1. pull docker images from azure to the basestation
        subt deployer local.ugv.ugv1.docker.registry.azure.pull

        # step 2. pull the docker images from the basestation to the robots
        subt deployer robots.ugv.ugv1.ppc.docker.registry.basestation.pull
        subt deployer robots.ugv.ugv1.nuc.docker.registry.basestation.pull
        subt deployer robots.ugv.ugv1.xavier.docker.registry.basestation.pull

## 2. Docker Containers

**Create Docker Containers**

Follow these steps, **on the basestation**.

        # stop any previous robot containers
        subt deployer robots.ugv.ugv1.ppc.core.docker.shell.stop
        subt deployer robots.ugv.ugv1.nuc.core.docker.shell.stop
        subt deployer robots.ugv.ugv1.xavier.core.docker.shell.stop

        # stop any previous robot containers
        subt deployer robots.ugv.ugv1.ppc.core.docker.shell.rm
        subt deployer robots.ugv.ugv1.nuc.core.docker.shell.rm
        subt deployer robots.ugv.ugv1.xavier.core.docker.shell.rm

        # create all the ugv docker containers on all computers
        subt deployer robots.ugv.ugv1.ppc.core.docker.shell.start
        subt deployer robots.ugv.ugv1.nuc.core.docker.shell.start
        subt deployer robots.ugv.ugv1.xavier.core.docker.shell.start
