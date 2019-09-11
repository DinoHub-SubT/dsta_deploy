# Basestation GUI

**Table Of Contents**

[TOC]


## Basestation GUI Build & Launch

1. **Clone the Basestation GUI Workspace**

        # Initializes the basestation rosinstall
        ./deployer -s basestation.rosinstall.gui

        # wstool updates
        ./deployer -s basestation.up.gui


2. **Build the docker image**

        ./deployer -s basestation.docker.gui.image

3. **Start the docker container**

        ./deployer -s basestation.docker.gui.start

4. **Build the Basestation Workspace**

        ./deployer -s basestation.gui.build

5. **Launch the Basestation Workspace**

        ./deployer -s basestation.gui.launch.start

6. **Source the workspace**

        echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
        echo "export ROS_HOSTNAME=[hostname]" >> ~/.bashrc
        # source launch workspace devel path
        echo "source ~/deploy_ws/devel/basestation/launch/setup.bash" >> ~/.bashrc

7. **Stop the Basestation Workspace Launch**

        ./deployer -s basestation.gui.launch.stop

8. **Stop the Docker Container**

        ./deployer -s basestation.docker.gui.stop

