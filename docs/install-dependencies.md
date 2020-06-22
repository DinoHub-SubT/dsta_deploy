# Install Requirements

## Container Provisioning Tools

### Docker

1. Remove old versions of Docker

        sudo apt-get remove docker docker-engine docker.io

2. Install dependencies and keys

        sudo apt install curl apt-transport-https ca-certificates curl software-properties-common

3. Add the official GPG key of Docker

        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add - 
        sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"


4. Install Docker

        sudo apt-get update && sudo apt-get install docker-ce


5. Add your user to the docker group:

        sudo usermod -a -G docker $USER

    - logout-log-back-in for the changes to take effect


6. Verify your Docker installation

        # verify docker is installed (without sudo access)
        docker -v

    - Do not run with `sudo docker`. Go back to Step 5 if you still cannot run as a non-root user.

7. Try running a sample container

        sudo docker run hello-world

      - You should see the message *Hello from Docker!* confirming that your installation was successfully completed.

### Docker Compose

1. Download current stable release of *docker compose*

        sudo apt-get update
        sudo apt-get install -y --no-install-recommends curl
        sudo curl -L "https://github.com/docker/compose/releases/download/1.24.1/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose

2. Apply executable permissions to the binary

        sudo chmod +x /usr/local/bin/docker-compose
        sudo ln -s /usr/local/bin/docker-compose /usr/bin/docker-compose

3. Test docker compose install

        docker-compose --version

### NVIDIA Docker (Optional)

**Proceed with the below instructions ONLY if you have a NVidia GPU.**

- The instructions below assumes you already [installed](https://askubuntu.com/a/1056128) an nvidia driver.

- For more information about installing `nvidia-docker`, please see the [official documentation](https://github.com/NVIDIA/nvidia-docker)

#### With Docker Version >= 19.03

For more information about installing nvidia-docker, for docker version >- `19.03` please see the [nvidia-docker official documentation](https://github.com/NVIDIA/nvidia-docker)

- `nvidia-docker2` is deprecated, so `nvidia-docker2` packages will no longer be supported.

- NVIDIA GPUs are now natively supported as devices in the Docker runtime, so you will not need to install `nvidia-docker`

1. Remove old version of Nvidia Docker

        docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
        sudo apt-get purge nvidia-docker nvidia-docker2

2. Setup the NVIDIA Docker Repository

        curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
        distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
        curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

3. Install NVIDIA Docker

        sudo apt-get update
        sudo apt-get install -y nvidia-container-toolkit

4. Restart the Docker daemon

        sudo systemctl restart docker

5. Try running a sample container

        docker run --gpus all nvidia/cuda:10.0-base nvidia-smi

    - The docker image `nvidia/cuda` requires a recent CUDA version. If you have an earlier CUDA version, then [find a tag](https://hub.docker.com/r/nvidia/cuda/tags) with an earlier version.
        - Example: `docker run --gpus all --rm nvidia/cuda:8.0-runtime nvidia-smi`

    - This command should print your GPU information.

#### With Docker Version < 19.03

For more information about installing nvidia-docker, for docker version < `19.03` please see the [nvidia-docker2 official documentation](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0))

1. Remove old version of Nvidia Docker

        docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
        sudo apt-get purge nvidia-docker nvidia-docker2

2. Install NVIDIA Docker

        sudo apt-get install -y nvidia-docker2

3. Restart the Docker daemon

        sudo service docker restart

4. Verify your Docker installation

        # verify docker is installed (without sudo access)
        nvidia-docker -v

5. Try running a sample container

        docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi

    - The docker image `nvidia/cuda` requires a recent CUDA version. If you have an earlier CUDA version, then [find a tag](https://hub.docker.com/r/nvidia/cuda/tags) with an earlier version.
        - Example: `docker run --runtime=nvidia --rm nvidia/cuda:8.0-runtime nvidia-smi`

    - This command should print your GPU information.

6. If NVIDIA docker fails to run, with the following error message: `Unknown runtime specified nvidia`


    - Systemd drop-in file

            sudo mkdir -p /etc/systemd/system/docker.service.d
            sudo tee /etc/systemd/system/docker.service.d/override.conf <<EOF
            [Service]
            ExecStart=
            ExecStart=/usr/bin/dockerd --host=fd:// --add-runtime=nvidia=/usr/bin/nvidia-container-runtime
            EOF
            sudo systemctl daemon-reload
            sudo systemctl restart docker

    - Daemon configuration file

            sudo tee /etc/docker/daemon.json <<EOF
            {
            "runtimes": {
                    "nvidia": {
                    "path": "/usr/bin/nvidia-container-runtime",
                    "runtimeArgs": []
                    }
            }
            }
            EOF
            sudo pkill -SIGHUP dockerd

    - Try NVIDIA runntime argument again:

            docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi

## Cloud Provisioning Tools

Install the following third-party cloud provisioning operational tools.

### Azure CLI

        # Dependencies
        sudo apt-get update
        sudo apt-get install ca-certificates curl apt-transport-https lsb-release gnupg

        # Software Signing Key
        curl -sL https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/microsoft.asc.gpg > /dev/null

        # Add Azure CLI Software Repository
        AZ_REPO=$(lsb_release -cs)
        echo "deb [arch=amd64] https://packages.microsoft.com/repos/azure-cli/ $AZ_REPO main" | sudo tee /etc/apt/sources.list.d/azure-cli.list

        # Azure CLI Package
        sudo apt-get update
        sudo apt-get install azure-cli

### Terraform

        # Dependencies
        sudo apt-get install unzip wget

        # Terraform CLI package
        cd ~/Downloads/
        wget https://releases.hashicorp.com/terraform/0.12.24/terraform_0.12.24_linux_amd64.zip
        unzip terraform_0.12.24_linux_amd64.zip
        sudo mv terraform /usr/local/bin/
        rm terraform_0.12.24_linux_amd64.zip

### Ansible

        # Add ansible to install
        sudo apt update
        sudo apt install software-properties-common
        sudo apt-add-repository --yes --update ppa:ansible/ansible

        # Install ansible
        sudo apt install ansible

### TeamViewer (optional)

        # Download the teamviewer deb package
        cd /tmp/
        wget https://download.teamviewer.com/download/linux/teamviewer_amd64.deb

        # Install teamviewer
        # At the prompt Do you want to continue? [Y/n], type Y to continue the installation.
        sudo apt install ./teamviewer_amd64.deb

        # Remove the deb package
        rm teamviewer_amd64.deb

- For more reference, please see [here](https://linuxize.com/post/how-to-install-teamviewer-on-ubuntu-18-04/).

## Verify Installations

Verify you have all the third-party operations tools installed correctly:

        # verify docker
        docker --version

        # verify docker-compose
        docker-compose -v

        # verify nvidia-docker
        nvidia-docker -v

        # verify ansible configuration management tools
        ansible --version

        # verify terraform cloud provisioning tool
        terraform --version

        # verify azure cli
        az --help

        # teamviewer client for remote VM desktop access
        teamviewer --help

Verify you can run the deployer scripts installed correctly:

        # source your bashrc (or zshrc)
        source ~/.bashrc

        # verify docker-compose shows the help usage message
        docker-compose-wrapper --help

        # verify deployer script shows the help usage message
        ./deployer --help

Notify the maintainer if any of the `help` usage messages do not show up.
