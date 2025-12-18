#!/usr/bin/env bash

# The set -e option instructs bash to immediately exit if any command has a non-zero exit status.
# The set -u option causes the bash shell to treat unset variables as an error and exit immediately.
# The set -o pipefail option prevents errors in a pipeline from being masked.
set -euo pipefail

# -----------------------------------------------------------------------------
# setup
# -----------------------------------------------------------------------------
OS="$(uname -s)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/devcontainer_environment_variables"

# -----------------------------------------------------------------------------
# helper functions
# -----------------------------------------------------------------------------
log_info() { echo -e "[INFO]  $*"; }
log_warn() { echo -e "[WARN]  $*"; }
log_error() { echo -e "[ERROR] $*" >&2; }
separator() { echo "------------------------------------------------------------"; }

setup() {
	local display_value="$1"
	shift
	local lines=("$@")

	if [[ -f "$ENV_FILE" ]]; then
		log_info "Environment file $ENV_FILE already exists. Skipping."
	else
		log_info "Writing environment variables to $ENV_FILE"
		{
			echo "devcontainer_environment_variables"
			echo
			echo "DISPLAY=$display_value"
			echo "USER=${USER:-unknown}"
		} >"$ENV_FILE"
	fi

	local shell_name
	shell_name=$(basename "$SHELL" 2>/dev/null || echo "bash")
	log_info "Detected shell: $shell_name"

	local profile_file
	case "$shell_name" in
	zsh) profile_file="$HOME/.zshrc" ;;
	bash) profile_file="$HOME/.bashrc" ;;
	*) profile_file="$HOME/.profile" ;;
	esac

	log_info "Using $profile_file"
	touch "$profile_file"

	for line in "${lines[@]}"; do
		if ! grep -qxF "$line" "$profile_file"; then
			echo "$line" >>"$profile_file"
			log_info "Added '$line' to $profile_file"
		else
			log_info "'$line' already exists in $profile_file"
		fi
	done
}

# -----------------------------------------------------------------------------
# linux setup
# -----------------------------------------------------------------------------
setup_linux() {
	log_info "Detected Linux environment."

	# install X11 tools
	log_info "Installing X11 utilities..."
	sudo apt-get update -qq
	sudo apt-get install -y x11-utils x11-xserver-utils

	# gpu detection
	if command -v nvidia-smi &>/dev/null; then
		log_info "NVIDIA GPU detected."
		setup ":0" 'export DOCKER_GPU_RUN_ARGS="--runtime=nvidia"' 'export DOCKER_RUNTIME_RUN_ARGS="--gpus=all"'

		if ! command -v nvidia-ctk &>/dev/null; then
			log_info "Installing NVIDIA Container Toolkit..."

			curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey |
				sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit.gpg

			if [[ ! -f /etc/os-release ]]; then
				log_error "/etc/os-release not found. Cannot determine distribution."
				exit 1
			fi

			. /etc/os-release
			distribution="${ID}${VERSION_ID:-}"
			case "$distribution" in
			ubuntu24.04)
				log_warn "ubuntu24.04 not yet supported by NVIDIA. Using ubuntu22.04 repo."
				distribution="ubuntu22.04"
				;;
			ubuntu2*)
				log_warn "Future Ubuntu release detected ($distribution). Falling back to ubuntu22.04."
				distribution="ubuntu22.04"
				;;
			debian1*)
				if [[ "$distribution" != "debian11" && "$distribution" != "debian12" ]]; then
					log_warn "Unsupported Debian release ($distribution). Falling back to debian12."
					distribution="debian12"
				fi
				;;
			esac
			log_info "Using NVIDIA repository for: $distribution"

			repo_url="https://nvidia.github.io/libnvidia-container/${distribution}/libnvidia-container.list"
			repo_tmp=$(mktemp)
			if curl -fsSL "$repo_url" -o "$repo_tmp"; then
				if grep -q "^deb " "$repo_tmp"; then
					sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit.gpg] https://#g' \
						"$repo_tmp" | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
					log_info "NVIDIA repo list installed successfully."
				else
					log_error "Unexpected repo content; aborting."
					cat "$repo_tmp"
					exit 1
				fi
			else
				log_error "Failed to download NVIDIA repo for distribution: ${distribution:-unknown}"
				log_error "URL: ${repo_url}"
				log_error "Check your internet connection or ensure this distro is supported by NVIDIA."
				exit 1
			fi
			rm -f "$repo_tmp"

			export DOCKER_GPU_RUN_ARGS="--runtime=nvidia"
			export DOCKER_RUNTIME_RUN_ARGS="--gpus=all"

			sudo apt-get update
			sudo apt-get install -y nvidia-container-toolkit
			log_info "Configuring Docker runtime for NVIDIA..."
			sudo nvidia-ctk runtime configure --runtime=docker
			sudo systemctl restart docker

		else
			log_info "NVIDIA Container Toolkit already installed."
		fi

	else
		log_info "No NVIDIA GPU found. Running CPU-only mode."
		setup ":0"
	fi
}

# -----------------------------------------------------------------------------
# macOS setup
# -----------------------------------------------------------------------------
setup_macos() {
	log_info "Detected macOS environment."

	if ! command -v brew &>/dev/null; then
		log_warn "Please install Homebrew and then XQuartz for proper X11 support."
		exit 1
	else
		if brew list --cask xquartz &>/dev/null; then
			log_info "XQuartz is already installed via Homebrew."
		else
			log_info "Installing XQuartz via Homebrew..."
			brew install --cask xquartz
		fi
	fi

	setup "docker.for.mac.host.internal:0"
	log_warn "GPU passthrough not supported on Docker Desktop for macOS."
}

# -----------------------------------------------------------------------------
# Fallback for unknown OS
# -----------------------------------------------------------------------------
setup_unknown() {
	log_warn "Unsupported OS detected: $OS"
	setup ":0"
	log_warn "Running CPU-only. Display may not work properly."
}

separator
case "$OS" in
Linux) setup_linux ;;
Darwin) setup_macos ;;
*) setup_unknown ;;
esac
separator

log_info "Setup complete!"
exit 0
