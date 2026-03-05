sudo docker login

# Images for AMD/ Intel CPUs
docker build -t vtautoboat/development_image --platform linux/amd64 -f .devcontainer/Dockerfile .
docker build -t vtautoboat/development_image_microros --platform linux/amd64 -f .devcontainer/devcontainer_variants/Dockerfile.microros .
docker build -t vtautoboat/development_image_deepstream --platform linux/amd64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream .


sudo docker push vtautoboat/development_image:latest
sudo docker push vtautoboat/development_image_microros
sudo docker push vtautoboat/development_image_deepstream






# # Images for ARM CPUs or the Jetson
docker build -t vtautoboat/development_image:arm --platform linux/arm64 -f .devcontainer/Dockerfile .
docker build -t vtautoboat/development_image_microros:arm --platform linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.microros .
docker build -t vtautoboat/development_image_deepstream:arm --platform linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream .


sudo docker push vtautoboat/development_image:arm
sudo docker push vtautoboat/development_image_microros:arm
sudo docker push vtautoboat/development_image_deepstream:arm



# docker buildx build -t vtautoboat/development_image --platform linux/amd64, linux/arm64 -f .devcontainer/Dockerfile .
# docker buildx build -t vtautoboat/development_image_microros --platform linux/amd64, linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.microros .
# docker buildx build -t vtautoboat/development_image_deepstream --platform linux/amd64, linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream .
