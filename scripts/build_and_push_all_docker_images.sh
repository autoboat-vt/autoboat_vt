# Usage: scripts/build_and_push_all_docker_images.sh
# You don't have to run this entire script and you can really just pick and choose which images you want to build
# by copy pasting the commands into your command line

echo "Login with the team's docker hub account"
sudo docker login

# Images for AMD/ Intel CPUs
docker build -t vtautoboat/development_image_base --platform linux/amd64 -f .devcontainer/Dockerfile .
docker build -t vtautoboat/development_image_firmware --platform linux/amd64 -f .devcontainer/devcontainer_variants/Dockerfile.firmware_dependencies .
docker build -t vtautoboat/development_image_deepstream --platform linux/amd64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream .


sudo docker push vtautoboat/development_image_base:latest
sudo docker push vtautoboat/development_image_firmware
sudo docker push vtautoboat/development_image_deepstream






# # Images for ARM CPUs or the Jetson
docker build -t vtautoboat/development_image_base:arm --platform linux/arm64 -f .devcontainer/Dockerfile .
docker build -t vtautoboat/development_image_firmware:arm --platform linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.firmware_dependencies .
docker build -t vtautoboat/development_image_deepstream:arm --platform linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream .


sudo docker push vtautoboat/development_image_base:arm
sudo docker push vtautoboat/development_image_firmware:arm
sudo docker push vtautoboat/development_image_deepstream:arm



# docker buildx build -t vtautoboat/development_image_base --platform linux/amd64, linux/arm64 -f .devcontainer/Dockerfile .
# docker buildx build -t vtautoboat/development_image_firmware --platform linux/amd64, linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.firmware_dependencies .
# docker buildx build -t vtautoboat/development_image_deepstream --platform linux/amd64, linux/arm64 -f .devcontainer/devcontainer_variants/Dockerfile.deepstream .
