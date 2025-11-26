docker build -t vtautoboat/autoboat_docker_dev_image -f .devcontainer/Dockerfile .
docker build -t vtautoboat/autoboat_docker_dev_image:microros -f .devcontainer/devcontainer_variants/Dockerfile.microros .
docker build -t vtautoboat/autoboat_docker_dev_image:deepstream -f .devcontainer/devcontainer_variants/Dockerfile.deepstream .
docker build -t vtautoboat/autoboat_docker_dev_image:yolo -f .devcontainer/devcontainer_variants/Dockerfile.yolo .
docker build -t vtautoboat/autoboat_docker_dev_image:deepstream_and_yolo -f .devcontainer/devcontainer_variants/Dockerfile.deepstream_and_yolo .