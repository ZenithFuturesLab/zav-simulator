#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == --dry-run ]]; then
    echo 'Ubuntu 22.04: ROS 2 Humble desktop, Fast/Cyclone DDS, colcon, rosdep, C++/PX4/Agent dependencies'
    exit 0
fi
# Never leave package configuration waiting for a GUI/terminal dialog.
apt_run() {
    sudo env DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=l apt-get \
        -o Acquire::Retries=3 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30 "$@"
}
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || { echo 'Supported: Ubuntu 22.04 (native or WSL2)' >&2; exit 1; }
apt_run update
apt_run install -y curl ca-certificates gnupg locales software-properties-common
sudo locale-gen en_US.UTF-8
sudo add-apt-repository -y universe
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    repo_deb=$(mktemp --suffix=.deb)
    trap 'rm -f "$repo_deb"' EXIT
    curl -fL --connect-timeout 15 --max-time 180 --retry 3 -o "$repo_deb" https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.jammy_all.deb
    sudo dpkg -i "$repo_deb"
fi
apt_run update
apt_run install -y ros-humble-desktop ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp ros-humble-ament-cmake-auto ros-humble-pcl-conversions \
    ros-humble-rosbag2 ros-humble-cv-bridge python3-colcon-common-extensions python3-rosdep \
    python3-pip python3-venv python3-numpy python3-scipy python3-yaml python3-opencv \
    python3-jinja2 python3-empy python3-toml python3-packaging python3-jsonschema \
    python3-kconfiglib python3-future python3-serial python3-setuptools \
    git build-essential cmake ninja-build pkg-config ccache tmux libeigen3-dev \
    libopencv-dev libpcl-dev libboost-all-dev libapr1-dev libaprutil1-dev libasio-dev libtinyxml2-dev \
    libssl-dev libxml2-utils libncurses-dev xsltproc zip unzip rsync
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update --rosdistro humble
