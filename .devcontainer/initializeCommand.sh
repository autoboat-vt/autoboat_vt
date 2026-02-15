# ---------------------------------------------------------------------------------------------------------
# These commands are run right on the host computer before the docker container starts up
# ---------------------------------------------------------------------------------------------------------

# ensure that this finishes even if the command fails (the user doesn't have xhost)
xhost +local: || true  


# we would like to only pull the development image if we are rebuilding the devcontainer (which is equivalent to creating a new container).
# we can check this, by checking if there is a docker container on your computer with the name autoboat_dev_container.
# we name our devcontainer "autoboat_dev_container" which is specified in the run args of the devcontainer.json.
# when you rebuild a devcontainer, it then deletes the current container and starts to rebuild a new container,
# but when you are just reopening a container, then the container that you are attempting to open already exists and therefore we use the following command to sense that.
if docker container inspect autoboat_dev_container >/dev/null 2>&1; then
    echo "Container autoboat_dev_container exists."
else
    echo "Container autoboat_dev_container does not exist."
    docker pull --platform=linux/amd64 vtautoboat/autoboat_docker_dev_image:simulation_testing
fi
