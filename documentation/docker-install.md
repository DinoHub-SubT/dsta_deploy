
# Docker Install

**Table Of Contents**

[TOC]


## System Requirements

- *At minimum*, Ubuntu 16.04 installed
- Internet connection
- **If installing nvidia-docker**, Nvidia drivers already installed on your host machine.

## Install Docker

1. Remove old versions of Docker

    `sudo apt-get remove docker docker-engine docker.io`

2. Install dependencies and keys

    `sudo apt install curl apt-transport-https ca-certificates curl software-properties-common`

3. Add the official GPG key of Docker

        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add - 
        sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"


4. Install Docker

    `sudo apt-get update && sudo apt-get install docker-ce`


5. Add your user to the docker group:

    `sudo usermod -a -G docker $USER`

    - logout-log-back-in for the changes to take effect


6. Verify your Docker installation

    * **Please** do not run with `sudo docker`. Go back to Step 5 if you still cannot run as a non-root user.


    *To verify if `docker` is installed:*

    `docker -v`

    *Try running a sample container:*

    `sudo docker run hello-world`

    - You should see the message *Hello from Docker!* confirming that your installation was successfully completed.

## Install Nvidia Docker

* **Proceed with the below instructions only if you have a NVidia GPU.**

1. Remove old version of Nvidia Docker

    `docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f`

2. Install Nvidia Docker

    `sudo apt-get purge -y nvidia-docker`

3. Setup the Nvidia Docker Repository

        curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
        distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
        curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
        sudo apt-get update

4. Install Nvidia Docker (version 2):

        sudo apt-get install -y nvidia-docker2

5. Restart the Docker daemon

        sudo service docker restart

6. Verify the installation:

    *To verify if `nvidia-docker` is installed:*

    `nvidia-docker -v`

    *Try running a sample container:*

    `docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi`

    - The docker image `nvidia/cuda` requires a recent CUDA version. If you have an earlier CUDA version, then [find a tag](https://hub.docker.com/r/nvidia/cuda/tags) with an earlier version.
        - Example: `docker pull nvidia/cuda:8.0-runtime` and then run the `docker run` command with the `nvidia/cuda:8.0-runtime` image.

    - This command should print your GPU information.

[[top]](#markdown-header-deployer)