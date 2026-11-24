#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$repo_dir/scripts/webcam_platform.sh"
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
  echo 'This installer requires Ubuntu 22.04 (native or WSL2).' >&2
  exit 1
fi
if (( EUID == 0 )); then
  echo 'Run the installer as your regular user. It uses sudo for Ubuntu packages.' >&2
  exit 1
fi

if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --v4l2 ) ]]; then
  echo 'Usage: ./install_usb_webcam.sh [--v4l2]' >&2
  exit 2
fi
packages=(ros-humble-ros-base)
if webcam_is_wsl; then
  if [[ $(uname -m) != x86_64 ]]; then
    echo 'Windows camera capture requires x86_64 WSL2.' >&2
    exit 1
  fi
  packages+=(ca-certificates curl unzip ros-humble-rmw-cyclonedds-cpp)
fi
if ! webcam_is_wsl || [[ ${1:-} == --v4l2 ]]; then
  packages+=(ros-humble-usb-cam v4l-utils)
fi

temp_dir=$(mktemp -d)
trap 'rm -rf -- "$temp_dir"' EXIT
missing=()
for package in "${packages[@]}"; do
  if [[ $(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true) != 'install ok installed' ]]; then
    missing+=("$package")
  fi
done
if (( ${#missing[@]} )); then
  sudo apt-get update
  sudo apt-get install -y ca-certificates curl software-properties-common
  sudo add-apt-repository -y --no-update universe
  if ! apt-cache show ros-humble-ros-base >/dev/null 2>&1; then
    ros_repo=https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.jammy_all.deb
    curl -fL --retry 3 "$ros_repo" -o "$temp_dir/ros2-apt-source.deb"
    sudo dpkg -i "$temp_dir/ros2-apt-source.deb"
  fi
  sudo apt-get update
  sudo apt-get install -y "${missing[@]}"
fi

if webcam_is_wsl; then
  if ! command -v powershell.exe >/dev/null || ! ffmpeg=$(webcam_ffmpeg_path); then
    echo 'Windows interop is required for WSL webcam capture.' >&2
    exit 1
  fi
  if [[ ! -x "$ffmpeg" ]]; then
    archive=https://www.gyan.dev/ffmpeg/builds/ffmpeg-release-essentials.zip
    curl -fL --retry 3 "$archive" -o "$temp_dir/ffmpeg.zip"
    sha=$(curl -fsSL --retry 3 "$archive.sha256" | awk 'NR==1 {print $1}')
    printf '%s  %s\n' "$sha" "$temp_dir/ffmpeg.zip" | sha256sum -c -
    unzip -j "$temp_dir/ffmpeg.zip" '*/bin/ffmpeg.exe' -d "$temp_dir"
    install -D -m 755 "$temp_dir/ffmpeg.exe" "$ffmpeg"
  fi
  "$ffmpeg" -hide_banner -version >/dev/null
fi

printf 'Webcam setup complete. Run:\n  %q\n' "$repo_dir/run_usb_webcam.sh"
