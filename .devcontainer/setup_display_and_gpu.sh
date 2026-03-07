#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# IMPORTANT NOTE
# This script is the main thing that is limiting devcontainer compatibility with any
# distribution of linux or operating system. Unfortunately it is really annoying to fully 
# support all linux distributions and all package managers because some of the 
# package managers don't even have the packages we need.
#
# The current devcontainer compatibility with linux distros/ package managers is as follows:
# Ubuntu/ Debain/ Mint based (apt): Normal + GPU forwarding
# Fedora/ RHEL based (dnf): Normal 
# RHEL/ CentOS based (yum): Normal
# Arch Linux based (pacman): Normal
# Alpine Linux based (apk): Normal
#
# GPU forwarding support means that the devcontainer will be able to see your GPU which
# may be useful for testing deepstream accelerated computer vision tasks inside of the
# devcontainer. 
#
# Currently Ubuntu/ Debian/ Mint based distros have the most robust support as most
# users currently end up using some sort of Debian inspired distro. But if you end up
# using another distro and you would like better support or the support listed here 
# has deteriorated, please reach out to me at chrisjnassif@gmail.com and I can help you out.
# -----------------------------------------------------------------------------



# The set -e option instructs bash to immediately exit if any command has a non-zero exit status.
# The set -u option causes the bash shell to treat unset variables as an error and exit immediately.
# The set -o pipefail option prevents errors in a pipeline from being masked.
set -euo pipefail


# Make the script ask the user for their linux/ shell password
sudo -v

# -----------------------------------------------------------------------------
# Setup
# -----------------------------------------------------------------------------
OS="$(uname -s)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/devcontainer_environment_variables"


# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------
log_info() { echo -e "[INFO]  $*"; }
log_warn() { echo -e "[WARN]  $*"; }
log_error() { echo -e "[ERROR] $*" >&2; }
print_separator() { echo "------------------------------------------------------------"; }



install_x11_linux() {
    # Installs all relevant x11 packages depending on the installed package manager and updates package cache
    if command -v apt &> /dev/null; then
        echo "Detected Debian/Ubuntu (apt). Installing packages: x11-utils, x11-xserver-utils"
        sudo apt-get update -qq || true 
        sudo apt-get install -y -qq x11-utils x11-xserver-utils
        
    elif command -v dnf &> /dev/null; then
        echo "Detected Fedora/RHEL 8+ (dnf). Installing packages: xorg-x11-utils, xorg-x11-server-utils"
        sudo dnf makecache -q
        sudo dnf install -y xorg-x11-utils xorg-x11-server-utils
        
    elif command -v yum &> /dev/null; then
        echo "Detected RHEL/CentOS (yum). Installing packages: xorg-x11-utils, xorg-x11-server-utils"
        sudo yum makecache -q
        sudo yum install -y xorg-x11-utils xorg-x11-server-utils
        
    elif command -v pacman &> /dev/null; then
        echo "Detected Arch Linux (pacman). Installing xorg-apps group."
        sudo pacman -Syu --noconfirm xorg-apps
        
    elif command -v apk &> /dev/null; then
        echo "Detected Alpine Linux (apk). Installing individual X11 utilities."
        # Alpine requires specifying the exact tools normally found in the Ubuntu metapackages
        sudo apk add --update \
            xdpyinfo xev xfontsel xkill xlsatoms xlsclients xlsfonts xmessage \
			xprop xvinfo xwininfo appres editres viewres iceauth rgb sessreg xgamma \
			xhost xmodmap xrandr xrdb xrefresh xset xsetroot xvidtune
        
    else
        echo "Error: No supported package manager found. Cannot install packages automatically."
        exit 1
    fi
}



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

	local profile_file;
	local rc_file;
	case "$shell_name" in
	zsh) profile_file="$HOME/.zprofile"; rc_file="$HOME/.zshrc" ;;
	bash) profile_file="$HOME/.bash_profile"; rc_file="$HOME/.bashrc"  ;;
	*) log_error "Unrecognized shell environment: ${shell_name}."; exit 1;;
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


	log_info "Using $rc_file"
	touch "$rc_file"

	for line in "${lines[@]}"; do
		if ! grep -qxF "$line" "$rc_file"; then
			echo "$line" >>"$rc_file"
			log_info "Added '$line' to $rc_file"
		else
			log_info "'$line' already exists in $rc_file"
		fi
	done
}



# -----------------------------------------------------------------------------
# linux setup
# -----------------------------------------------------------------------------
setup_linux() {
	log_info "Detected Linux environment."
	
	# Make sure that the user has proper permissions on the entire repository
	sudo chmod -R 777 ..

	# install X11 tools
	log_info "Installing X11 utilities..."
	install_x11_linux


	# Load udev rules for each device and remove any autoboat udev rules that existed before
	# Udev rules basically just rename devices for us so that they are easier to access
	# For example the raspberry pi pico udev rule would just make it so that the file descriptor for the raspberry pi pico device
	# Would always be at the file: /dev/pico
	# You can read more about udev rules here: https://opensource.com/article/18/11/udev
	sudo chmod -R 777 /etc/udev/

	if [ -f "/etc/udev/rules.d/99-autoboat-udev.rules" ]; then
	    sudo rm -f /etc/udev/rules.d/99-autoboat-udev.rules
	    sudo rm -f /etc/udev/rules.d/99-autoboat-udev.rules
	fi

	sudo echo 'ACTION=="add", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", SYMLINK+="pico", MODE="0666"' >> /etc/udev/rules.d/99-autoboat-udev.rules
	sudo echo 'ACTION=="add", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK+="gps", MODE="0666"' >> /etc/udev/rules.d/99-autoboat-udev.rules
	# sudo echo 'ACTION=="add", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", SYMLINK+="camera", MODE="0666"' >> /etc/udev/rules.d/99-autoboat-udev.rules
	sudo echo 'ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A9001WL3", SYMLINK+="rc", MODE="0666"' >> /etc/udev/rules.d/99-autoboat-udev.rules
	sudo echo 'ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="ABSCDYAB", SYMLINK+="wind_sensor", MODE="0666"' >> /etc/udev/rules.d/99-autoboat-udev.rules

	sudo udevadm control --reload-rules
	sudo udevadm trigger



	# GPU detection
	if command -v nvidia-smi &>/dev/null && command -v apt &> /dev/null; then
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
				# TODO: I believe this is no longer necessary as nvidia now supports Ubuntu 24.04
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

			sudo apt update -qq || true
			sudo apt install -y nvidia-container-toolkit
			log_info "Configuring Docker runtime for NVIDIA..."
			sudo nvidia-ctk runtime configure --runtime=docker
			sudo systemctl restart docker

		else
			log_info "NVIDIA Container Toolkit already installed."
		fi

	elif command -v nvidia-smi &>/dev/null; then
		log_warn "NVIDIA GPU Detected, but you are not running a linux distribution that supports devcontainer GPU forwarding. Running CPU-only mode."
	

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



print_separator
case "$OS" in
Linux) setup_linux ;;
Darwin) setup_macos ;;
*) setup_unknown ;;
esac
print_separator

log_info "Setup complete!"
exit 0