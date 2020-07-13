# About Deployer

* * *

## Overview

`deployer` is a python script that reads configuration yamls and executes the commands listed in the deploy yamls on the remote or local host.

All deploy yamls are located in `ci/deploy/`. All field deployment deploy yamls are located in `ci/deploy/field`.

## Workflow

**Your workflow might look like:**

- Update the dockerfiles with new dependencies
   - build the docker images on the *local* system
- Start the docker container on the *local* system
- Update the rosinstalls or update the submodules on the *local* system
    - wstool update the new rosinstall updated repos in the docker container
- Transfer the deploy repository from the *local* system to the *remote* system
- Build the docker images on the remote system
- Start the remote docker container
- Build the repositories, in the docker container docker, on the remote workspace
- Launch, in the docker container, the remote workspace

All these workflow steps should already have pre-configured deploy yamls setup.
****
   - The user only needs to run the `deployer`.

   - If no deploy yamls are present or have errors, notify the [maintainer](maintainer.md).

* * *

## Deploy Yamls

> Deploy yamls configure robot deployment tasks.

- **Example Usage:**

    `./deployer -s basestation.build.planning`

    - runs the `build.planning` yaml section on the basestation


    `./deployer -s basestation.launch.planning.start -p -v`

    - previews the `launch.planning.start` yaml section on the basestation


### Overview

Running the deployer may look like: `./deployer -s basestation.build.planning`

- The deployer **matches command line configuration key words to the same keyword in a yaml file**.
    
    - Key words are called *phases*

- Deployer looks for any yaml file, in `ci/deploy/field/`, that matches the keyword and stops at the first found match.

    - deploy yamls are located in `ci/deploy/field/`
    
    - Yaml filenames do not need to match any keywords. Deployer matches the phase keywords *inside* the yaml.

- **Yaml Terminology**

    Deploy       | Description
    ------------- | -------------
    Step | a single bash command
    Phase | keyword that groups sequential steps
    Section | a group of phases

    - Largely inspired by [Travis CI](https://docs.travis-ci.com/user/for-beginners/).

### Specifics

A phase is a dictionary *key* in the yaml that describes the sequence of shell commands to run.

- **Deploy Yaml Example:**

        - echo "start here"
        - basestation:
          - echo "hello world"
          - docker:
            - echo "docker commands to run"
          - build:
            - echo "build commands to run"
            - planning:
              - echo "build planning workspace"
        - echo "end here"

    Deploy       | Examples
    ------------- | -------------
    Step | `echo "start here"`, `echo "hello world"`
    Phase | *keywords at any level:* **basestation, docker, build, planning**
    Section | *keywords with phase children:* **basestation, build**

**What gets executed?**

- Execution is expanded as a depth first search pattern.
  - All steps (i.e. shell commands) on the path get executed.
- Keywords can be nested.
- A keyword **with** children, is a section.
- **Section** will run:
    - all its children: steps, sections phases.
    - all same level steps, that are not inside another section.
    - all previous levels or next steps, that are not inside another section.
- A keyword **without** children, is a phase.
- **Phase** will run:
    - all its steps
    - all same level steps, that are not inside another section.
    - all previous level or next steps, that are not inside another section.

**Example output executions**

- `./deployer -s basestation`

        echo "start here"
        echo "hello world"
        echo "docker commands to run"
        echo "build commands to run"
        echo "build planning workspace"
        echo "end here"

- `./deployer -s basestation.docker`
        
        echo "start here"
        echo "hello world"
        echo "docker commands to run"
        echo "end here"

- `./deployer -s basestation.build.planning` or `./deployer -s basestation.planning`
        
        echo "start here"
        echo "hello world"
        echo "build commands to run"
        echo "build planning workspace"
        echo "end here"

**How to match?**

- Section or phase names can be fully specified or shortly specified.
  - CLI keywords are pattern matched expanded to match the full section name.
  - The user can remove any intermediate section keyword.
    - Example: `basestation.build.planning`, `basestation.planning`, `basestation.build` will all match the same section.

[[top]](#markdown-header-about-deployer)

* * *