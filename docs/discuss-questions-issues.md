# Common Questions

- **When to re-build docker images?**

    - When docker image does not exist on the host ( run `docker images` on the localhost or VM to verify)

    - When new repository dependencies are added to dockerfiles (dockerfiles found in: `operations/deploy/docker/dockerfiles`)

- **When to update dockerfiles?**

    - You should make changes to dockerfiles when you want to need new to add dependencies for workspace repositories (example ros packages, linux packages, etc.). See the existing dockerfiles, found in: `operations/deploy/docker/dockerfiles`, for example dependencies.

    - If you install a dependency in the container directly, remember to put it in the dockerfile and rebuild the docker image.

- **Where can I add directories to mount inside docker containers**

    - Open `operations/deploy/docker/docker-compose.yml`

    - You will find the `base` docker compose tag.

    - Add another directory volume path right under: `${SUBT_PATH}/../:/home/developer/deploy_ws/:rw`

    - The format is: `localhost/path/to/directory:/home/developer/path/in/conttainer:rw`

    - remove the current container `docker rm [container name]`

    - restart the container shell (please use `subt deployer [TAB]` commands as appropriate).