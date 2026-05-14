# Docker Bake configuration for Autoboat VT Dev Images
#
# Variables are overridden by environment variables named BUILDX_BAKE_VAR_<NAME>

variable "OWNER" {
  default = "autoboat-vt"
}

variable "ARCHITECTURE" {
  default = "amd64"
}

variable "VERSION" {
  default = "0.0.0"
}

variable "MAJOR" {
  default = "0"
}

variable "MINOR" {
  default = "0.0"
}

variable "GITHUB_SHA" {
  default = "latest"
}

variable "IS_OFFICIAL" {
  default = "false"
}

variable "IS_RELEASE" {
  default = "false"
}

group "default" {
  targets = ["base", "firmware"]
}

target "base" {
  context = "."
  dockerfile = ".devcontainer/Dockerfile"
  platforms = ["linux/${ARCHITECTURE}"]
  tags = [
    "${IS_OFFICIAL}" == "true" ? "vtautoboat/development_image:${ARCHITECTURE}" : "vtautoboat/development_image:local-${ARCHITECTURE}",
    "${IS_OFFICIAL}" == "true" ? "ghcr.io/${OWNER}/development_image:${ARCHITECTURE}" : "",
    IS_OFFICIAL == "true" ? "ghcr.io/${OWNER}/development_image:${GITHUB_SHA}" : "",
    IS_OFFICIAL == "true" ? "ghcr.io/${OWNER}/development_image:${substr(GITHUB_SHA, 0, 7)}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image:v${VERSION}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image:v${MINOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image:v${MAJOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image:v${VERSION}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image:v${MINOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image:v${MAJOR}-${ARCHITECTURE}" : ""
  ]
}

target "firmware" {
  contexts = {
    base = "target:base"
  }
  dockerfile = ".devcontainer/devcontainer_variants/Dockerfile.firmware_sdk"
  platforms = ["linux/${ARCHITECTURE}"]
  args = {
    BASE_IMAGE = "base"
  }
  tags = [
    "${IS_OFFICIAL}" == "true" ? "vtautoboat/development_image_firmware:${ARCHITECTURE}" : "vtautoboat/development_image_firmware:local-${ARCHITECTURE}",
    "${IS_OFFICIAL}" == "true" ? "ghcr.io/${OWNER}/development_image_firmware:${ARCHITECTURE}" : "",
    IS_OFFICIAL == "true" ? "ghcr.io/${OWNER}/development_image_firmware:${GITHUB_SHA}" : "",
    IS_OFFICIAL == "true" ? "ghcr.io/${OWNER}/development_image_firmware:${substr(GITHUB_SHA, 0, 7)}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_firmware:v${VERSION}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_firmware:v${MINOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_firmware:v${MAJOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_firmware:v${VERSION}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_firmware:v${MINOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_firmware:v${MAJOR}-${ARCHITECTURE}" : ""
  ]
}
