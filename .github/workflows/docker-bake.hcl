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
  targets = ["base", "microros"]
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

target "microros" {
  contexts = {
    base = "target:base"
  }
  dockerfile = ".devcontainer/devcontainer_variants/Dockerfile.microros"
  platforms = ["linux/${ARCHITECTURE}"]
  args = {
    BASE_IMAGE = "base"
  }
  tags = [
    "${IS_OFFICIAL}" == "true" ? "vtautoboat/development_image_microros:${ARCHITECTURE}" : "vtautoboat/development_image_microros:local-${ARCHITECTURE}",
    "${IS_OFFICIAL}" == "true" ? "ghcr.io/${OWNER}/development_image_microros:${ARCHITECTURE}" : "",
    IS_OFFICIAL == "true" ? "ghcr.io/${OWNER}/development_image_microros:${GITHUB_SHA}" : "",
    IS_OFFICIAL == "true" ? "ghcr.io/${OWNER}/development_image_microros:${substr(GITHUB_SHA, 0, 7)}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_microros:v${VERSION}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_microros:v${MINOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "vtautoboat/development_image_microros:v${MAJOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_microros:v${VERSION}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_microros:v${MINOR}-${ARCHITECTURE}" : "",
    IS_RELEASE == "true" ? "ghcr.io/${OWNER}/development_image_microros:v${MAJOR}-${ARCHITECTURE}" : ""
  ]
}
