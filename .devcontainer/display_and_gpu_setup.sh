#!/usr/bin/env bash
set -euo pipefail

OS="$(uname -s)"
EXTRA_ARGS=""

if [[ "$OS" == "Linux" ]]; then
    
    # SETUP THE DISPLAY ENVIRONMENT VARIABLES FOR LINUX/ WSL
    echo -e "devcontainer_environment_variables\n\nDISPLAY=:0" > .devcontainer/devcontainer_environment_variables

    # SETUP THE GPU SUPPORT FOR LINUX/ WSL
    if command -v nvidia-smi &>/dev/null; then
        echo "[INFO] NVIDIA GPU detected. Ensuring nvidia-container-toolkit is installed..."

        if ! command -v nvidia-ctk &>/dev/null; then
            echo "[INFO] Installing NVIDIA Container Toolkit..."

            # Add GPG key
            curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
              | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit.gpg

            # Detect distro
            . /etc/os-release
            distribution="${ID}${VERSION_ID}"

            # --- Fallback mapping for unsupported distros ---
            case "$distribution" in
                ubuntu24.04)
                    echo "[WARN] ubuntu24.04 not yet supported by NVIDIA. Falling back to ubuntu22.04 repo."
                    distribution="ubuntu22.04"
                    ;;
                ubuntu2*)
                    echo "[WARN] Future Ubuntu release detected ($distribution). Falling back to ubuntu22.04 repo."
                    distribution="ubuntu22.04"
                    ;;
                debian1*)
                    if [[ "$distribution" != "debian11" && "$distribution" != "debian12" ]]; then
                        echo "[WARN] Unsupported Debian release ($distribution). Falling back to debian12 repo."
                        distribution="debian12"
                    fi
                    ;;
            esac

            echo "[INFO] Using distribution: $distribution"

            # Fetch repo list
            repo_url="https://nvidia.github.io/libnvidia-container/${distribution}/libnvidia-container.list"
            echo "[INFO] Fetching repo file: $repo_url"

            repo_tmp=$(mktemp)
            if curl -fsSL "$repo_url" -o "$repo_tmp"; then
                if grep -q "^deb " "$repo_tmp"; then
                    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit.gpg] https://#g' \
                        "$repo_tmp" | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
                    echo "[INFO] Repo list installed."
                else
                    echo "[ERROR] Repo URL returned unexpected content (not a deb line)."
                    cat "$repo_tmp"
                    exit 1
                fi
            else
                echo "[ERROR] Failed to download repo list for $distribution"
                echo "Try using a supported distro (e.g. ubuntu20.04, ubuntu22.04, debian11, debian12)."
                exit 1
            fi

            # Install toolkit
            sudo apt-get update
            sudo apt-get install -y nvidia-container-toolkit

            echo "[INFO] Configuring Docker runtime..."
            sudo nvidia-ctk runtime configure --runtime=docker
            sudo systemctl restart docker
        else
            echo "[INFO] NVIDIA Container Toolkit already installed."
        fi



        export DOCKER_RUNTIME_RUN_ARGS="--runtime=nvidia --gpus=all"
        echo 'export DOCKER_RUNTIME_RUN_ARGS="--runtime=nvidia --gpus=all"' >> ~/.bashrc
        echo 'export DOCKER_RUNTIME_RUN_ARGS="--runtime=nvidia --gpus=all"' >> ~/.profile


    else
        echo "[INFO] No GPU found, running CPU-only."
    fi


elif [[ "$OS" == "Darwin" ]]; then


    # SETUP THE DISPLAY ENVIRONMENT VARIABLES FOR MACOS
    echo -e "devcontainer_environment_variables\n\nDISPLAY=docker.for.mac.host.internal:0" >> .devcontainer/devcontainer_environment_variables
    
    echo "[INFO] macOS detected. GPU passthrough not supported in Docker Desktop."


else
    # SETUP THE DISPLAY ENVIRONMENT VARIABLES FOR AN UNKNOWN OS
    echo -e "devcontainer_environment_variables\n\nDISPLAY=:0" > .devcontainer/devcontainer_environment_variables
    
    echo "[WARN] Unsupported OS: $OS. Running CPU-only and Display may not work properly."
fi
