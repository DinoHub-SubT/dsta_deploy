# Basestation Planning

**Table Of Contents**

[TOC]


## Basestation Planning Build & Launch

1. **Clone the Basestation GUI Workspace**

        # Initializes the basestation rosinstall
        ./deployer -s basestation.rosinstall.planning

        # wstool updates
        ./deployer -s basestation.up.planning


2. **Build the docker image**

        ./deployer -s basestation.docker.planning.image

3. **Start the docker container**

        ./deployer -s basestation.docker.planning.start

4. **Build the Basestation Workspace**

        ./deployer -s basestation.planning.build

5. **Launch the Basestation Workspace**

        ./deployer -s basestation.planning.launch.start

6. **Source the workspace**

        echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
        echo "export ROS_HOSTNAME=[hostname]" >> ~/.bashrc
        # source launch workspace devel path
        echo "source ~/deploy_ws/devel/basestation/launch/setup.bash" >> ~/.bashrc

7. **Stop the Basestation Workspace Launch**

        ./deployer -s basestation.planning.launch.stop

8. **Stop the Docker Container**

        ./deployer -s basestation.docker.planning.stop
