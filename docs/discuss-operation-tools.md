# Operational Tools Discussion and Issues

## Operational Tools & Utilities

There are a few operational tools available to use:

`docker`

  - command interface to interact with `dockerfiles` found in `operations/deploy/docker/dockerfiles`

`docker-compose`

  - command interface to interact with `docker-compose.yml` files found in `operations/deploy/docker/dockerfiles/`

`docker-compose-wrapper`

  - command interface to wrap `docker compose` and with `scenario` configuration files found in `operations/deploy/scenarios`

`docker context`

  - command interface to manage multiple docker endpoints *(example: docker engine running on Azure VMs)*.

`deployer`

  - command interface to interact with `deployerfiles` found in `operations/deploy/deploybooks/`
  - automates running the tutorial steps, with realtime command output.

`ansible`

  - command interface to interact with `ansible playbooks` found in `operations/deploy/robotbooks/`
  - automates installing dependencies and setting up systems.


`ssh-connect-check`

  - tests ssh connection to hosts listed in the local `~/.ssh/config`

![Alt text](images/ssh-config-example.png?raw=true "Title")


## Deployer Tool

You should now have a built `SubT` workspace, either on the localhost or on an Azure setup.

You should become more familiar with the operational tools and their purpose of operations:

### Deployer Tool Syntax

**Template: Creating Docker Images**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # Create the docker image
        ./deployer -s [deployment host].[(optional) robot].[(optional) computer].docker.image

        # example: create a docker image on ugv1 Azure VM  (assumes ssh connection available)
        ./deployer -s azure.ugv1.docker.image

**Template: Creating Docker Shell Access Containers**

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # Create the docker shell container
        ./deployer -s [deployment host].[(optional) robot].[(optional) computer].docker.shell

        # example: create a docker container with shell access, on ugv1 Azure VM (assumes ssh connection available)
        ./deployer -s azure.ugv1.docker.shell

**Template: Building the catkin workspace**

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

**How to interact with the deployer tool**

To learn more about the available `deployer` tool commands, use the `--preview` or `-p` command as shown below:

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # preview ugv options when deploying on the remote Azure VM
        ./deployer -s azure.ugv1 -p

To learn more what the command executes, use the `--verbose` or `-v` option with the `preview` option as shown below:

        # go to the deploy top level path
        cd ~/deploy_ws/src

        # preview and verbosely display all the commands that will be executed on the Azure VM
        ./deployer -s azure.ugv1 -p -v

# Thirdparty Tools

**Some Helpful Tools For Remote Development**

- Improving the shell experience: `zsh`, `oh-my-zsh`
- Managing remote VM desktops: `rdp`, `teamviewer`
- Managing launch setups: `tmux`, `byobu`
- Remote desktop extensions on IDE, for example the [visual code plugin](https://code.visualstudio.com/docs/remote/remote-overview).
- Managing docker endpoints tools: `docker context`, `docker machine`, `docker swarm`

# Common Questions

- **When to re-build docker images?**

    - When docker image does not exist on the host ( run `docker images` on the localhost or VM to verify)

    - When new repository dependencies are added to dockerfiles (dockerfiles found in: `operations/deploy/docker/dockerfiles`)

- **When to update dockerfiles?**

    - You should make changes to dockerfiles when you want to need new to add dependencies for workspace repositories (example ros packages, linux packages, etc.). See the existing dockerfiles, found in: `operations/deploy/docker/dockerfiles`, for example dependencies.

    - If you install a dependency in the container directly, remember to put it in the dockerfile and rebuild the docker image.