# Robot UGV Docker Setup

Docker install all the repository dependencies as *docker images*.

- The docker images will exist on the ugv robots.

Docker shell containers will give the user access to the entire deploy workspace inside a docker container.

- You should be able to do anything inside the docker container that you would do normally do on the host.

All deployer commands should be done on the **basestation**.

## 1. Docker Images

**Create Docker Images**

Follow these steps, **on the basestation**.

        # azure registry login
        az acr login --name subtexplore

        # -- (ROBTS HAVE INTERNET) -- pull the docker images from the azure docker registry
        subt deployer robots.ugv.ugv1.docker.registry.azure.pull

        # (optional) run on a specific ugv robot computer
        subt deployer robots.ugv.ugv1.ppc.docker.registry.azure.pull
        subt deployer robots.ugv.ugv1.nuc.docker.registry.azure.pull
        subt deployer robots.ugv.ugv1.xavier.docker.registry.azure.pull

        # -- (ROBTS DO NOT HAVE INTERNET) -- pull the docker images from the basestation docker registry

        # step 1. pull docker images from azure to the basestation
        subt deployer local.ugv.ugv1.docker.registry.azure.pull

        # step 2 (optiona). pull the docker images from the basestation to the robots
        subt deployer robots.ugv.ugv1.docker.registry.basestation.pull

**Verify Docker Images**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh ugv.ppc

        # View the docker images built on the remote VM
        docker images

Verify you see the following docker images (in any order):

        # planning-pc (ppc)
        subt/x86.ugv.ppc.cpu.core                               0.2.c40347f
        subt/x86.ugv.ppc.cpu.ros.melodic                        0.2.c40347f

        # nuc
        subt/x86.ugv.nuc.cpu.core                               0.2.c40347f
        subt/x86.ugv.nuc.cpu.slam                               0.2.c40347f
        subt/x86.ugv.nuc.cpu.superodometry                      0.2.c40347f
        subt/x86.ugv.nuc.cpu.ros.melodic                        0.2.c40347f

        # xavier
        subt/arm.ugv.xavier.cpu.ros.melodic                     0.2.c40347f

Return To Localhost

        # exit the remote VM
        exit

## 2. Docker Containers

**Create Docker Containers**

Follow these steps, **on the basestation**.

        # create all the ugv docker containers on all computers
        subt deployer robots.ugv.ugv1.docker.shell.start

        # (optional) run on a specific ugv robot computer
        subt deployer robots.ugv.ugv1.ppc.docker.shell.start
        subt deployer robots.ugv.ugv1.nuc.docker.shell.start
        subt deployer robots.ugv.ugv1.xavier.docker.shell.start

**Verify Docker Containers**

        # ssh into your VM (if not already done so), change the below command to match your VM ssh access
        ssh ugv1.ppc

        # view running docker containers
        docker ps

Verify you see the following docker containers (in any order):

        ugv1-shell
        ugv1-perception-shell

Return To Localhost

        # exit the remote VM
        exit
