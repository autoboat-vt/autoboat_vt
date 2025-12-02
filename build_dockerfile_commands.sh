

# Images for AMD/ Intel CPUs
docker build -t vtautoboat/autoboat_docker_dev_image --platform linux/amd64 -f .devcontainer/Dockerfile .
docker build -t vtautoboat/autoboat_docker_dev_image:microros --platform linux/amd64 -f .devcontainer/devcontainer_variants/Dockerfile.microros .
docker build -t vtautoboat/autoboat_docker_dev_image:deepstream --platform linux/amd64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream .
docker build -t vtautoboat/autoboat_docker_dev_image:yolo --platform linux/amd64 -f .devcontainer/devcontainer_variants/Dockerfile.yolo .
docker build -t vtautoboat/autoboat_docker_dev_image:deepstream_and_yolo --platform linux/amd64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream_and_yolo .

# Images for ARM CPUs or the Jetson
docker build -t vtautoboat/autoboat_docker_dev_image --platform linux/arm64 -f .devcontainer/Dockerfile .
docker build -t vtautoboat/autoboat_docker_dev_image:microros --platform linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.microros .
docker build -t vtautoboat/autoboat_docker_dev_image:deepstream --platform linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream .
docker build -t vtautoboat/autoboat_docker_dev_image:yolo --platform linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.yolo .
docker build -t vtautoboat/autoboat_docker_dev_image:deepstream_and_yolo --platform linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream_and_yolo .