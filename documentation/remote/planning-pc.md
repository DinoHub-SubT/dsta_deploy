# Basestation GUI

**Table Of Contents**

[TOC]


## Planning-PC Build & Launch

1. **Clone the Basestation Perception Workspace**

        # Initializes the basestation rosinstall
        ./deployer -s basestation.rosinstall

        # wstool updates
        ./deployer -s basestation.up

2. Transfer from the basestation to the planning-pc

        ./deployer -s [robot].planning-pc.transfer.to
  
3. **Build the docker image**

        ./deployer -s [robot].planning-pc.docker.image

4. **Start the docker container**

        ./deployer -s [robot].planning-pc.docker.start

5. **Build the Basestation Workspace**

        ./deployer -s [robot].planning-pc.build

6. **Launch the Basestation Workspace**

        ./deployer -s [robot].planning-pc.launch.start

7. **Stop the Basestation Workspace Launch**

        ./deployer -s [robot].planning-pc.launch.stop

8.  **Stop the Docker Container**

        ./deployer -s [robot].planning-pc.docker.start
