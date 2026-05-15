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


# Build Base Debian package
echo "==> Building standard .deb package (v${DEB_VERSION} ${DEB_ARCH})..."
PKG_DIR_BASE="${REPOSITORY_ROOT_DIRECTORY}/deb_pkg_std"
mkdir -p "${PKG_DIR_BASE}/DEBIAN"
mkdir -p "${PKG_DIR_BASE}/opt/autoboat/"

cp -r /opt/autoboat/install "${PKG_DIR_BASE}/opt/autoboat/"

sed -e "s/VERSION_PLACEHOLDER/${DEB_VERSION}/" -e "s/ARCH_PLACEHOLDER/${DEB_ARCH}/" \
  .github/workflows/debian_package_files/control.template > "${PKG_DIR_BASE}/DEBIAN/control"

cp .github/workflows/debian_package_files/base/postinst "${PKG_DIR_BASE}/DEBIAN/postinst"
cp .github/workflows/debian_package_files/base/prerm   "${PKG_DIR_BASE}/DEBIAN/prerm"
cp .github/workflows/debian_package_files/base/postrm   "${PKG_DIR_BASE}/DEBIAN/postrm"
chmod 0755 "${PKG_DIR_BASE}/DEBIAN/postinst" "${PKG_DIR_BASE}/DEBIAN/prerm" "${PKG_DIR_BASE}/DEBIAN/postrm"



# Build Simulation package by moving all simulation files/ binaries from PKG_DIR_BASE to PKG_DIR_SIM
# Once the simulation stuff is removed and transplanted into PKG_DIR_BASE then we can compile both packages
# Simulation is amd64 only and if we are on arm then we don't have to worry about simulation binaries because they aren't built
if [ "${DEB_ARCH}" == "amd64" ]; then

  echo "==> Building simulation .deb package (v${DEB_VERSION} ${DEB_ARCH})..."
  PKG_DIR_SIM="${REPOSITORY_ROOT_DIRECTORY}/deb_pkg_sim"
  mkdir -p "${PKG_DIR_SIM}/DEBIAN"
  mkdir -p "${PKG_DIR_SIM}/opt/autoboat/install"
  
  SIM_PACKAGES="simulation_models custom_lift_drag foil_dynamics sail_limits simulation_transform wind_arrow"
  for PKG in $SIM_PACKAGES; do
    echo "Partitioning simulation package: ${PKG}"
    
    # Share (Launch files, models, etc.)
    if [ -d "${PKG_DIR_BASE}/opt/autoboat/install/share/${PKG}" ]; then
      mkdir -p "${PKG_DIR_SIM}/opt/autoboat/install/share"
      mv "${PKG_DIR_BASE}/opt/autoboat/install/share/${PKG}" "${PKG_DIR_SIM}/opt/autoboat/install/share/"
    fi
    
    # Lib (Python executables and C++ binaries)
    if [ -d "${PKG_DIR_BASE}/opt/autoboat/install/lib/${PKG}" ]; then
      mkdir -p "${PKG_DIR_SIM}/opt/autoboat/install/lib"
      mv "${PKG_DIR_BASE}/opt/autoboat/install/lib/${PKG}" "${PKG_DIR_SIM}/opt/autoboat/install/lib/"
    fi
    
    # Python site-packages
    PYTHON_SITE="${PKG_DIR_BASE}/opt/autoboat/install/lib/python3.10/site-packages"
    if [ -d "${PYTHON_SITE}/${PKG}" ]; then
      mkdir -p "${PKG_DIR_SIM}/opt/autoboat/install/lib/python3.10/site-packages"
      mv "${PYTHON_SITE}/${PKG}" "${PKG_DIR_SIM}/opt/autoboat/install/lib/python3.10/site-packages/"
    fi

    # Shared Libraries (lib<pkg>.so)
    find "${PKG_DIR_BASE}/opt/autoboat/install/lib" -maxdepth 1 -name "lib${PKG}.so*" -exec mv {} "${PKG_DIR_SIM}/opt/autoboat/install/lib/" \; 2>/dev/null || true
  done
  
  # Control file
  sed -e "s/VERSION_PLACEHOLDER/${DEB_VERSION}/" -e "s/ARCH_PLACEHOLDER/${DEB_ARCH}/" \
      .github/workflows/debian_package_files/control-simulation.template > "${PKG_DIR_SIM}/DEBIAN/control"
  
  cp .github/workflows/debian_package_files/base/prerm "${PKG_DIR_SIM}/DEBIAN/prerm"
  chmod 0755 "${PKG_DIR_SIM}/DEBIAN/prerm"

  dpkg-deb -Zzstd --build "${PKG_DIR_SIM}" "output_artifacts/autoboatvt-simulation-${DEB_ARCH}.deb"
fi

dpkg-deb -Zzstd --build "${PKG_DIR_BASE}" "output_artifacts/autoboatvt-${DEB_ARCH}.deb"




# Microros Agent package
echo "==> Building microros-agent .deb package (v${DEB_VERSION} ${DEB_ARCH})..."
PKG_DIR_AGENT="${REPOSITORY_ROOT_DIRECTORY}/deb_pkg_agent"
mkdir -p "${PKG_DIR_AGENT}/DEBIAN"
mkdir -p "${PKG_DIR_AGENT}/opt/autoboat/firmware_dependencies"

cp -r /opt/autoboat/firmware_dependencies/micro_ros_agent "${PKG_DIR_AGENT}/opt/autoboat/firmware_dependencies/"

sed -e "s/VERSION_PLACEHOLDER/${DEB_VERSION}/" -e "s/ARCH_PLACEHOLDER/${DEB_ARCH}/" \
  .github/workflows/debian_package_files/control-microros-agent.template > "${PKG_DIR_AGENT}/DEBIAN/control"

cp .github/workflows/debian_package_files/microros_agent/postinst "${PKG_DIR_AGENT}/DEBIAN/postinst"
cp .github/workflows/debian_package_files/microros_agent/postrm   "${PKG_DIR_AGENT}/DEBIAN/postrm"
chmod 0755 "${PKG_DIR_AGENT}/DEBIAN/postinst" "${PKG_DIR_AGENT}/DEBIAN/postrm"

dpkg-deb -Zzstd --build "${PKG_DIR_AGENT}" "output_artifacts/autoboatvt-microros-agent-${DEB_ARCH}.deb"




# Firmware SDK Debian package
echo "==> Building firmware-dependencies .deb package (v${DEB_VERSION} ${DEB_ARCH})..."
PKG_DIR_FIRMWARE="${REPOSITORY_ROOT_DIRECTORY}/deb_pkg_uc"
mkdir -p "${PKG_DIR_FIRMWARE}/DEBIAN"
mkdir -p "${PKG_DIR_FIRMWARE}/opt/autoboat/firmware_dependencies"

cp -r /opt/autoboat/firmware_dependencies/micro_ros_raspberrypi_pico_sdk "${PKG_DIR_FIRMWARE}/opt/autoboat/firmware_dependencies/"
cp -r /opt/autoboat/firmware_dependencies/picotool "${PKG_DIR_FIRMWARE}/opt/autoboat/firmware_dependencies/"
cp -r /opt/autoboat/firmware_dependencies/pico-sdk "${PKG_DIR_FIRMWARE}/opt/autoboat/firmware_dependencies/"

echo "  Firmware SDK package size before compression: $(du -sh "${PKG_DIR_FIRMWARE}" | cut -f1)"

sed -e "s/VERSION_PLACEHOLDER/${DEB_VERSION}/" -e "s/ARCH_PLACEHOLDER/${DEB_ARCH}/" \
  .github/workflows/debian_package_files/control-firmware-dependencies.template > "${PKG_DIR_FIRMWARE}/DEBIAN/control"

cp .github/workflows/debian_package_files/firmware-dependencies/postinst "${PKG_DIR_FIRMWARE}/DEBIAN/postinst"
cp .github/workflows/debian_package_files/firmware-dependencies/postrm   "${PKG_DIR_FIRMWARE}/DEBIAN/postrm"
chmod 0755 "${PKG_DIR_FIRMWARE}/DEBIAN/postinst" "${PKG_DIR_FIRMWARE}/DEBIAN/prerm" "${PKG_DIR_FIRMWARE}/DEBIAN/postrm"

dpkg-deb --build "${PKG_DIR_FIRMWARE}" "output_artifacts/autoboatvt-firmware-dependencies-${DEB_ARCH}.deb"



# Fix permissions for host runner upload
echo "==> Fixing permissions for host runner..."
chmod -R a+rX output_artifacts/

echo "==> Output artifacts:"
du -sh output_artifacts/*

