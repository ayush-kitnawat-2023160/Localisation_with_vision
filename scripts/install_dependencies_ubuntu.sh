#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=""
else
  SUDO="sudo"
fi

if ! command -v lsb_release >/dev/null 2>&1; then
  echo "ERROR: lsb_release is required to detect the Ubuntu version." >&2
  exit 2
fi

UBUNTU_VERSION="$(lsb_release -rs)"
ROS_DISTRO="${ROS_DISTRO:-}"
if [[ -n "${ROS_DISTRO:-}" ]]; then
  ROS_DISTRO="${ROS_DISTRO}"
else
  for setup_file in /opt/ros/*/setup.bash; do
    if [[ -f "$setup_file" ]]; then
      ROS_DISTRO="$(basename "$(dirname "$setup_file")")"
      break
    fi
  done
fi

if [[ -z "$ROS_DISTRO" ]]; then
  echo "ERROR: no /opt/ros/<distro>/setup.bash found; install ROS1 first." >&2
  exit 2
fi

echo "Detected Ubuntu ${UBUNTU_VERSION}; ROS ${ROS_DISTRO}."
if [[ "$UBUNTU_VERSION" != "20.04" && "$UBUNTU_VERSION" != "18.04" ]]; then
  echo "WARNING: this package is most directly validated on Ubuntu 20.04/ROS Noetic."
  echo "         No OS upgrade will be attempted. Continue only if your ROS/PCL/OpenCV"
  echo "         toolchain provides the dependencies below."
fi

ROS_PREFIX="ros-${ROS_DISTRO}"
APT_PACKAGES=(
  "${ROS_PREFIX}-cmake-modules"
  "${ROS_PREFIX}-cv-bridge"
  "${ROS_PREFIX}-eigen-conversions"
  "${ROS_PREFIX}-image-transport"
  "${ROS_PREFIX}-pcl-conversions"
  "${ROS_PREFIX}-pcl-ros"
  "${ROS_PREFIX}-rosbag"
  "${ROS_PREFIX}-rviz"
  "${ROS_PREFIX}-tf"
  "${ROS_PREFIX}-tf2-ros"
  "${ROS_PREFIX}-geometry-msgs"
  "${ROS_PREFIX}-nav-msgs"
  "${ROS_PREFIX}-sensor-msgs"
  "${ROS_PREFIX}-std-msgs"
  libboost-thread-dev
  libeigen3-dev
  libopencv-dev
  libpcl-dev
  python3-numpy
)

echo "Installing only package dependencies; Docker and OS upgrades are intentionally not touched."
$SUDO apt-get update
$SUDO apt-get install -y "${APT_PACKAGES[@]}"

if [[ "$(uname -m)" =~ ^(x86_64|amd64)$ && -f "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/vendor/sophus_legacy/lib/libSophus.so" ]]; then
  echo "Using the bundled legacy x86_64 Sophus ABI. No system Sophus replacement is installed."
else
  echo "WARNING: bundled Sophus is x86_64-only. On this architecture provide a compatible"
  echo "         legacy Sophus installation and pass Sophus_DIR or SOPHUS_* to CMake."
fi

echo "Dependencies installed. Build with:"
echo "  cd $(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "  catkin_make"
