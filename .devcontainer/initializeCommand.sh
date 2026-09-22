#!/usr/bin/env bash

# ---------------------------------------------------------------------------------------------------------
# These commands are run on the host computer right before the docker container starts up
# ---------------------------------------------------------------------------------------------------------


if [ -d "/opt/X11/bin" ]; then
    export PATH="/opt/X11/bin:$PATH"
fi


HOST_OS="$(uname -s 2>/dev/null || echo Unknown)"
case "$HOST_OS" in
    Linux)
        xhost +local:root 2>/dev/null || true
        xhost +local:"${USER:-}" 2>/dev/null || true
        xhost +local: 2>/dev/null || true
        ;;
    Darwin)
        xhost + 127.0.0.1 2>/dev/null || true
        xhost + localhost 2>/dev/null || true
        ;;
    *)
        xhost +local: 2>/dev/null || true
        # Some unsupported OS.
        ;;
esac


# we would like to only pull the development image if we are rebuilding the devcontainer (which is equivalent to creating a new container).
# we can check this, by checking if there is a docker container on your computer with the name autoboat_dev_container.
# we name our devcontainer "autoboat_dev_container" which is specified in the run args of the devcontainer.json.
# when you rebuild a devcontainer, it then deletes the current container and starts to rebuild a new container,
# but when you are just reopening a container, then the container that you are attempting to open already exists and therefore we use the following command to sense that.
if docker container inspect autoboat_dev_container >/dev/null 2>&1; then
    echo "Container autoboat_dev_container exists."
else
    echo "Container autoboat_dev_container does not exist."
    docker pull --platform=linux/amd64 ${DEVCONTAINER_VARIANT:=vtautoboat/development_image_base}
fi
