# Operational Tools Discussion

## About Operational Tools & Utilities

There are a few operational tools available to use:

`docker`

  - command interface to interact with `dockerfiles` found in `operations/docker/dockerfiles`

`docker-compose`

  - command interface to interact with `docker-compose.yml` files found in `operations/docker/dockerfiles/`

`docker-compose-wrapper`

  - command interface to wrap `docker compose` and with `scenario` configuration files found in `operations/scenarios`

`docker context`

  - command interface to manage multiple docker endpoints *(example: docker engine running on Azure VMs)*.

`deployer`

  - command interface to interact with `deployerfiles` found in `operations/deploybooks/`
  - automates running the tutorial steps, with realtime command output.

`ansible`

  - command interface to interact with `ansible playbooks` found in `operations/robotbooks/`
  - automates installing dependencies and setting up systems.

`subt tools ssh`

  - tests ssh connection to hosts listed in the local `~/.ssh/config`

  - **Example output**:

```text
          # //////////////////////////////////////////////////////////////////////////////
          # == Testing SSH Connection ==
          # //////////////////////////////////////////////////////////////////////////////

          ugv1.ppc              FAIL
          ugv1.nuc              FAIL
          ugv1.xavier           FAIL
          azure.basestation     OK.
          azure.ugv1            OK.
          azure.ugv2            FAIL
          azure.ugv3            FAIL
          azure.uav1            OK.
          azure.uav2            FAIL
          azure.uav3            FAIL
          azure.uav4            FAIL
          azure.perception1     FAIL
```

`subt tools teamveiwer`

  - tests teamviewer connection to hosts listed in the local `~/.ssh/config`

  - **Example output**:

```text
          # //////////////////////////////////////////////////////////////////////////////
          # == Testing SSH Connection ==
          # //////////////////////////////////////////////////////////////////////////////

          ugv1.ppc              FAIL
          ugv1.nuc              FAIL
          ugv1.xavier           FAIL
          azure.basestation     1713206669
          azure.ugv1            1714515085
          azure.ugv2            FAIL
          azure.ugv3            FAIL
          azure.uav1            1710017259
          azure.uav2            FAIL
          azure.uav3            FAIL
          azure.uav4            FAIL
          azure.perception1     FAIL
```

`ccd`

  - changes current directory to the top level deploy src path.

`deploy-azure-limits-eastus`

  - shows vCPUs limits for *East US* region

  - **Example output**:
```text
          Total Regional vCPUs  995  1000
```

`deploy-azure-limits-eastus2`

  - shows vCPUs limits for *East US 2* region

  - **Example output**:

```text
          Total Regional vCPUs  31  350
```

`deploy-vpn-ca-cert`

  - outputs the `caCert` with the following command: `openssl x509 -in caCert.pem -outform der | base64 -w0 ; echo`

## Thirdparty Tools

**Some Helpful Tools For Remote Development**

- Improving the shell experience: `zsh`, `oh-my-zsh`
- Managing remote VM desktops: `rdp`, `teamviewer`
- Managing launch setups: `tmux`, `byobu`
- Remote desktop extensions on IDE, for example the [visual code plugin](https://code.visualstudio.com/docs/remote/remote-overview).
- Managing docker endpoints tools: `docker context`, `docker machine`, `docker swarm`

* * *

## Deployer Tool

You should now have a built `SubT` workspace, either on the localhost or on an Azure setup.

You should become more familiar with the operational tools and their purpose of operations:

### Deployer Tool Syntax

**Template: Creating Docker Images**

```text
# go to the deploy top level path
cd ~/deploy_ws/src

# Create the docker image
./deployer -s [deployment host].[(optional) robot].[(optional) computer].docker.image

# example: create a docker image on ugv1 Azure VM  (assumes ssh connection available)
./deployer -s azure.ugv1.docker.image
```

**Template: Creating Docker Shell Access Containers**

```text
# go to the deploy top level path
cd ~/deploy_ws/src

# Create the docker shell container
./deployer -s [deployment host].[(optional) robot].[(optional) computer].docker.shell

# example: create a docker container with shell access, on ugv1 Azure VM (assumes ssh connection available)
./deployer -s azure.ugv1.docker.shell
```

**Template: Building the catkin workspace**

```text
# (optional) Access the host
ssh azure.host-name

# enter the docker shell container
docker-join.bash --name [container name]

# go to the workspace path
cd ~/deploy_ws/src/path/to/workspace

# view the catkin profiles
catkin profile list

# select the catkin profile
catkin profile set [profile name]

# build the workspace
catkin build
```

**How to interact with the deployer tool**

To learn more about the available `deployer` tool commands, use the `--preview` or `-p` command as shown below:

```text
# go to the deploy top level path
cd ~/deploy_ws/src

# preview ugv options when deploying on the remote Azure VM
./deployer -s azure.ugv1 -p
```

To learn more what the command executes, use the `--verbose` or `-v` option with the `preview` option as shown below:

```text
# go to the deploy top level path
cd ~/deploy_ws/src

# preview and verbosely display all the commands that will be executed on the Azure VM
./deployer -s azure.ugv1 -p -v
```
