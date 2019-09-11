# Basestation State Estimation

**Table Of Contents**

[TOC]


## Basestation State Estimation Build & Launch

1. **Clone the Basestation GUI Workspace**

        # Initializes the basestation rosinstall
        ./deployer -s basestation.rosinstall.state_est

        # wstool updates
        ./deployer -s basestation.up.state_est


2. **Build the docker image**

        ./deployer -s basestation.docker.state_est.image

3. **Start the docker container**

        ./deployer -s basestation.docker.state_est.start

4. **Build the Basestation Workspace**

        ./deployer -s basestation.state_est.setup
        ./deployer -s basestation.state_est.build

5. **Launch the Basestation Workspace**

        ./deployer -s basestation.state_est.launch.start

6. **Source the workspace**

        echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
        echo "export ROS_HOSTNAME=[hostname]" >> ~/.bashrc
        # source launch workspace devel path
        echo "source ~/deploy_ws/devel/basestation/launch/setup.bash" >> ~/.bashrc

7. **Stop the Basestation Workspace Launch**

        ./deployer -s basestation.state_est.launch.stop

8. **Stop the Docker Container**

        ./deployer -s basestation.docker.state_est.stop

