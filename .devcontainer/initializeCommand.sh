xhost +localhost || true

if [ -s "/home/has_dockerfile_been_built" ]; then
    docker pull vtautoboat/autoboat_docker_dev_image
    echo "dockerfile has been built" >> /home/has_dockerfile_been_built 
fi
docker pull vtautoboat/autoboat_docker_dev_image