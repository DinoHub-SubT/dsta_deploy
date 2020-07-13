# NUC

**Table Of Contents**

[TOC]


## Nuc Build & Launch

1. **Clone the Basestation Perception Workspace**

        # Initializes the basestation rosinstall
        ./deployer -s basestation.rosinstall

        # wstool updates
        ./deployer -s basestation.up

2. Transfer from the basestation to the nuc

        ./deployer -s [robot].nuc.transfer.to
  
3. **Build the docker image**

        ./deployer -s [robot].nuc.docker.image

4. **Start the docker container**

        ./deployer -s [robot].nuc.docker.start

5. **Build the Basestation Workspace**

        ./deployer -s [robot].nuc.build

6. **Launch the Basestation Workspace**

        ./deployer -s [robot].nuc.launch.start

7. **Stop the Basestation Workspace Launch**

        ./deployer -s [robot].nuc.launch.stop

8.  **Stop the Docker Container**

        ./deployer -s [robot].nuc.docker.start
