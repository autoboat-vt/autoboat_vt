#!/bin/bash
set -e

# This script is intended to be run inside the ROS 2 development container.
# It handles building ROS 2 packages and generating multiple .deb packages.

# Expected Environment Variables:
# - DEB_VERSION: The version string for the package (e.g., 1.2.3)
# - DEB_ARCH: The target architecture (amd64 or arm64)
# - IGNORE_PACKAGES: Space-separated list of colcon packages to ignore

echo "==> Build Environment Info:"
echo "    Architecture: ${DEB_ARCH}"
echo "    Version:      ${DEB_VERSION}"
echo "    Ignore List:  ${IGNORE_PACKAGES}"

# Initialize ROS 2 environment
source /opt/ros/humble/setup.bash

# Use current directory as workspace root
REPOSITORY_ROOT_DIRECTORY=$(pwd)

echo "==> Starting colcon build..."
colcon build --packages-ignore ${IGNORE_PACKAGES} \
             --cmake-args -DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=mold" \
             --install-base /opt/autoboat/install

mkdir -p output_artifacts



# ── Standard Debian package (Runtime) ────────────────────────
echo "==> Building standard .deb package (v${DEB_VERSION} ${DEB_ARCH})..."
PKG_DIR_STD="${REPOSITORY_ROOT_DIRECTORY}/deb_pkg_std"
mkdir -p "${PKG_DIR_STD}/DEBIAN"
mkdir -p "${PKG_DIR_STD}/opt/autoboat"

# Copy built ROS 2 nodes
cp -r /opt/autoboat/install "${PKG_DIR_STD}/opt/autoboat/"

# Control file + Maintainer scripts
sed -e "s/VERSION_PLACEHOLDER/${DEB_VERSION}/" -e "s/ARCH_PLACEHOLDER/${DEB_ARCH}/" \
  .github/deb/control.template > "${PKG_DIR_STD}/DEBIAN/control"

cp .github/deb/base/postinst "${PKG_DIR_STD}/DEBIAN/postinst"
cp .github/deb/base/prerm      "${PKG_DIR_STD}/DEBIAN/prerm"
cp .github/deb/base/postrm   "${PKG_DIR_STD}/DEBIAN/postrm"
chmod 0755 "${PKG_DIR_STD}/DEBIAN/postinst" "${PKG_DIR_STD}/DEBIAN/prerm" "${PKG_DIR_STD}/DEBIAN/postrm"



# ── Simulation package logic (amd64 only) ──────────────────────
if [ "${DEB_ARCH}" == "amd64" ]; then
  echo "==> Building simulation .deb package (v${DEB_VERSION} ${DEB_ARCH})..."
  PKG_DIR_SIM="${REPOSITORY_ROOT_DIRECTORY}/deb_pkg_sim"
  mkdir -p "${PKG_DIR_SIM}/DEBIAN"
  mkdir -p "${PKG_DIR_SIM}/opt/autoboat/install"
  
  SIM_PACKAGES="simulation_models custom_lift_drag foil_dynamics custom_lift_drag sail_limits simulation_transform wind_arrow"
  for PKG in $SIM_PACKAGES; do
    echo "Partitioning simulation package: ${PKG}"
    
    # Share (Launch files, models, etc.)
    if [ -d "${PKG_DIR_STD}/opt/autoboat/install/share/${PKG}" ]; then
      mkdir -p "${PKG_DIR_SIM}/opt/autoboat/install/share"
      mv "${PKG_DIR_STD}/opt/autoboat/install/share/${PKG}" "${PKG_DIR_SIM}/opt/autoboat/install/share/"
    fi
    
    # Lib (Python executables and C++ binaries)
    if [ -d "${PKG_DIR_STD}/opt/autoboat/install/lib/${PKG}" ]; then
      mkdir -p "${PKG_DIR_SIM}/opt/autoboat/install/lib"
      mv "${PKG_DIR_STD}/opt/autoboat/install/lib/${PKG}" "${PKG_DIR_SIM}/opt/autoboat/install/lib/"
    fi
    
    # Python site-packages
    PYTHON_SITE="${PKG_DIR_STD}/opt/autoboat/install/lib/python3.10/site-packages"
    if [ -d "${PYTHON_SITE}/${PKG}" ]; then
      mkdir -p "${PKG_DIR_SIM}/opt/autoboat/install/lib/python3.10/site-packages"
      mv "${PYTHON_SITE}/${PKG}" "${PKG_DIR_SIM}/opt/autoboat/install/lib/python3.10/site-packages/"
    fi

    # Shared Libraries (lib<pkg>.so)
    find "${PKG_DIR_STD}/opt/autoboat/install/lib" -maxdepth 1 -name "lib${PKG}.so*" -exec mv {} "${PKG_DIR_SIM}/opt/autoboat/install/lib/" \; 2>/dev/null || true
  done
  
  # Control file
  sed -e "s/VERSION_PLACEHOLDER/${DEB_VERSION}/" -e "s/ARCH_PLACEHOLDER/${DEB_ARCH}/" \
      .github/deb/control-simulation.template > "${PKG_DIR_SIM}/DEBIAN/control"
      
  cp .github/deb/base/prerm "${PKG_DIR_SIM}/DEBIAN/prerm"
  chmod 0755 "${PKG_DIR_SIM}/DEBIAN/prerm"
  
  dpkg-deb -Zzstd --build "${PKG_DIR_SIM}" "output_artifacts/autoboat-vt-simulation-${DEB_ARCH}.deb"
fi

dpkg-deb -Zzstd --build "${PKG_DIR_STD}" "output_artifacts/autoboat-vt-${DEB_ARCH}.deb"



# ── Micro-ROS Agent Debian package ────────────────────────────
echo "==> Building microros-agent .deb package (v${DEB_VERSION} ${DEB_ARCH})..."
PKG_DIR_AGENT="${REPOSITORY_ROOT_DIRECTORY}/deb_pkg_agent"
mkdir -p "${PKG_DIR_AGENT}/DEBIAN"
mkdir -p "${PKG_DIR_AGENT}/opt/autoboat/microros_dependencies"

# Copy the micro_ros_agent workspace
cp -r /opt/autoboat/microros_dependencies/micro_ros_agent "${PKG_DIR_AGENT}/opt/autoboat/microros_dependencies/"

# Control file
sed -e "s/VERSION_PLACEHOLDER/${DEB_VERSION}/" -e "s/ARCH_PLACEHOLDER/${DEB_ARCH}/" \
  .github/deb/control-microros-agent.template > "${PKG_DIR_AGENT}/DEBIAN/control"

cp .github/deb/microros_agent/postinst "${PKG_DIR_AGENT}/DEBIAN/postinst"
cp .github/deb/microros_agent/postrm   "${PKG_DIR_AGENT}/DEBIAN/postrm"
chmod 0755 "${PKG_DIR_AGENT}/DEBIAN/postinst" "${PKG_DIR_AGENT}/DEBIAN/postrm"

dpkg-deb -Zzstd --build "${PKG_DIR_AGENT}" "output_artifacts/autoboat-vt-microros-agent-${DEB_ARCH}.deb"



# ── Micro-ROS SDK Debian package (Standalone Development) ────
echo "==> Building microros-sdk .deb package (v${DEB_VERSION} ${DEB_ARCH})..."
PKG_DIR_UC="${REPOSITORY_ROOT_DIRECTORY}/deb_pkg_uc"
mkdir -p "${PKG_DIR_UC}/DEBIAN"
mkdir -p "${PKG_DIR_UC}/opt/autoboat/microros_dependencies"

# Copy SDK dependencies for building microros packages (agent provided by autoboatvt-microros-agent)
cp -r /opt/autoboat/microros_dependencies/micro_ros_raspberrypi_pico_sdk "${PKG_DIR_UC}/opt/autoboat/microros_dependencies/"
cp -r /opt/autoboat/microros_dependencies/picotool "${PKG_DIR_UC}/opt/autoboat/microros_dependencies/"
cp -r /opt/autoboat/microros_dependencies/pico-sdk "${PKG_DIR_UC}/opt/autoboat/microros_dependencies/"

echo "  Microros package size before compression: $(du -sh "${PKG_DIR_UC}" | cut -f1)"

# Control file (standalone — no dependency on autoboatvt)
sed -e "s/VERSION_PLACEHOLDER/${DEB_VERSION}/" -e "s/ARCH_PLACEHOLDER/${DEB_ARCH}/" \
  .github/deb/control-microros.template > "${PKG_DIR_UC}/DEBIAN/control"

# Package-specific maintainer scripts
cp .github/deb/microros/postinst "${PKG_DIR_UC}/DEBIAN/postinst"
cp .github/deb/microros/postrm   "${PKG_DIR_UC}/DEBIAN/postrm"
chmod 0755 "${PKG_DIR_UC}/DEBIAN/postinst" "${PKG_DIR_UC}/DEBIAN/postrm"

# Use zstd compression (dramatically faster than default xz, similar ratio)
dpkg-deb -Zzstd --build "${PKG_DIR_UC}" "output_artifacts/autoboat-vt-microros-full-${DEB_ARCH}.deb"



# ── Fix permissions for host runner upload ───────────────────
echo "==> Fixing permissions for host runner..."
chmod -R a+rX output_artifacts/

echo "==> Output artifacts:"
du -sh output_artifacts/*

