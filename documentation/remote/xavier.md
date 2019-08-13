# Xavier

**Table Of Contents**

[TOC]


## xavier Build & Launch

1. **Clone the Basestation Perception Workspace**

        # Initializes the basestation rosinstall
        ./deployer -s basestation.rosinstall

        # wstool updates
        ./deployer -s basestation.up

2. Transfer from the basestation to the xavier

        ./deployer -s [robot].xavier.transfer.to
  
3. **Build the docker image**

        ./deployer -s [robot].xavier.docker.image

4. **Build the Basestation Workspace**

        ./deployer -s [robot].xavier.build

5. **Launch the Basestation Workspace**

        ./deployer -s [robot].xavier.launch.start

6. **Stop the Basestation Workspace Launch**

        ./deployer -s [robot].xavier.launch.stop
