#!/usr/bin/env bash
# Jalankan sekali di Ubuntu 24.04 (termasuk Jetson native) untuk ROS 2 Jazzy.
set -e

. /etc/os-release
if [ "${VERSION_ID:-}" != "24.04" ]; then
  echo "Script ini ditujukan untuk Ubuntu 24.04 (Noble)."
  exit 1
fi

sudo apt update
sudo apt install -y curl gnupg lsb-release software-properties-common
sudo add-apt-repository -y universe

# Tambahkan repository resmi ROS bila Jazzy belum tersedia dari apt.
if ! apt-cache show ros-jazzy-ros-base >/dev/null 2>&1; then
  sudo install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    sudo gpg --dearmor --yes -o /etc/apt/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt update
fi

sudo apt install -y build-essential cmake libopencv-dev \
    python3-opencv python3-rosdep \
    ros-jazzy-ros-base ros-jazzy-cv-bridge ros-jazzy-v4l2-camera \
    ros-jazzy-image-tools ros-jazzy-rqt-image-view \
    ros-jazzy-mavros ros-jazzy-mavros-extras \
    python3-colcon-common-extensions

curl -fsSLO https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh

echo "Setup selesai. Jalankan: source /opt/ros/jazzy/setup.bash"
