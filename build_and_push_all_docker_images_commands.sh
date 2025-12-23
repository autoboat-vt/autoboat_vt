sudo docker login

# Images for AMD/ Intel CPUs
docker build -t vtautoboat/autoboat_docker_dev_image:latest --platform linux/amd64 -f .devcontainer/Dockerfile .
docker build -t vtautoboat/autoboat_docker_dev_image:microros --platform linux/amd64 -f .devcontainer/devcontainer_variants/Dockerfile.microros .
docker build -t vtautoboat/autoboat_docker_dev_image:deepstream --platform linux/amd64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream .
docker build -t vtautoboat/autoboat_docker_dev_image:yolo --platform linux/amd64 -f .devcontainer/devcontainer_variants/Dockerfile.yolo .
docker build -t vtautoboat/autoboat_docker_dev_image:deepstream_and_yolo --platform linux/amd64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream_and_yolo .


docker push vtautoboat/autoboat_docker_dev_image:latest
docker push vtautoboat/autoboat_docker_dev_image:microros
docker push vtautoboat/autoboat_docker_dev_image:deepstream
docker push vtautoboat/autoboat_docker_dev_image:yolo
docker push vtautoboat/autoboat_docker_dev_image:deepstream_and_yolo






# # Images for ARM CPUs or the Jetson
docker build -t vtautoboat/autoboat_docker_dev_image:latest_arm --platform linux/arm64 -f .devcontainer/Dockerfile .
docker build -t vtautoboat/autoboat_docker_dev_image:microros_arm --platform linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.microros .
docker build -t vtautoboat/autoboat_docker_dev_image:deepstream_arm --platform linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream .
docker build -t vtautoboat/autoboat_docker_dev_image:yolo_arm --platform linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.yolo .
docker build -t vtautoboat/autoboat_docker_dev_image:deepstream_and_yolo_arm --platform linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream_and_yolo .


docker push vtautoboat/autoboat_docker_dev_image:latest_arm
docker push vtautoboat/autoboat_docker_dev_image:microros_arm
docker push vtautoboat/autoboat_docker_dev_image:deepstream_arm
docker push vtautoboat/autoboat_docker_dev_image:yolo_arm
docker push vtautoboat/autoboat_docker_dev_image:deepstream_and_yolo_arm



# docker buildx build -t vtautoboat/autoboat_docker_dev_image:latest --platform linux/amd64, linux/arm64 -f .devcontainer/Dockerfile .
# docker buildx build -t vtautoboat/autoboat_docker_dev_image:microros --platform linux/amd64, linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.microros .
# docker buildx build -t vtautoboat/autoboat_docker_dev_image:deepstream --platform linux/amd64, linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream .
# docker buildx build -t vtautoboat/autoboat_docker_dev_image:yolo --platform linux/amd64, linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.yolo .
# docker buildx build -t vtautoboat/autoboat_docker_dev_image:deepstream_and_yolo --platform linux/amd64, linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream_and_yolo .
