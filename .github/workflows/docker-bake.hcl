# Docker Bake configuration for Autoboat VT Dev Images
#
# Variables are overridden by environment variables named BUILDX_BAKE_VAR_<NAME>

variable "OWNER" {
  default = "autoboatvt"
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

variable "BASE_IMAGE" {
  default = "vtautoboat/development_image_base:latest"
}

group "default" {
  targets = ["base", "firmware"]
}

target "base" {
  context = "."
  dockerfile = ".devcontainer/Dockerfile"
  platforms = ["linux/${ARCHITECTURE}"]
  tags = [
    "${IS_OFFICIAL}" == "true" ? "vtautoboat/development_image_base:${ARCHITECTURE}" : "vtautoboat/development_image_base:local-${ARCHITECTURE}",
    "${IS_OFFICIAL}" == "true" ? "ghcr.io/${OWNER}/development_image_base:${ARCHITECTURE}" : "",
    IS_OFFICIAL == "true" ? "ghcr.io/${OWNER}/development_image_base:${GITHUB_SHA}" : "",
    IS_OFFICIAL == "true" ? "ghcr.io/${OWNER}/development_image_base:${substr(GITHUB_SHA, 0, 7)}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_base:v${VERSION}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_base:v${MINOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_base:v${MAJOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_base:v${VERSION}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_base:v${MINOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_base:v${MAJOR}-${ARCHITECTURE}" : ""
  ]
}

target "firmware" {
  contexts = {
    base = "target:base"
  }
  dockerfile = ".devcontainer/devcontainer_variants/Dockerfile.firmware_dependencies"
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

target "deepstream" {
  dockerfile = ".devcontainer/devcontainer_variants/Dockerfile.deepstream"
  platforms = ["linux/amd64"]
  pull = true
  args = {
    BASE_IMAGE = "${BASE_IMAGE}"
  }
  tags = [
    "${IS_OFFICIAL}" == "true" ? "vtautoboat/development_image_deepstream:latest" : "vtautoboat/development_image_deepstream:local-amd64",
    "${IS_OFFICIAL}" == "true" ? "vtautoboat/development_image_deepstream:${ARCHITECTURE}" : "",
    "${IS_OFFICIAL}" == "true" ? "ghcr.io/${OWNER}/development_image_deepstream:latest" : "",
    "${IS_OFFICIAL}" == "true" ? "ghcr.io/${OWNER}/development_image_deepstream:${ARCHITECTURE}" : "",
    IS_OFFICIAL == "true" ? "ghcr.io/${OWNER}/development_image_deepstream:${GITHUB_SHA}" : "",
    IS_OFFICIAL == "true" ? "ghcr.io/${OWNER}/development_image_deepstream:${substr(GITHUB_SHA, 0, 7)}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_deepstream:v${VERSION}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_deepstream:v${VERSION}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_deepstream:v${MINOR}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_deepstream:v${MINOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_deepstream:v${MAJOR}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_deepstream:v${MAJOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_deepstream:v${VERSION}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_deepstream:v${VERSION}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_deepstream:v${MINOR}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_deepstream:v${MINOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_deepstream:v${MAJOR}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_deepstream:v${MAJOR}-${ARCHITECTURE}" : ""
  ]
}

