#!/usr/bin/env bash
set -e

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$repo_dir/scripts/webcam_platform.sh"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Run ./install_usb_webcam.sh first.' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash

if webcam_is_wsl && [[ ${1:-} != /dev/* ]]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0
  export CYCLONEDDS_URI="file://$repo_dir/scripts/webcam_cyclonedds.xml"
  if [[ ${1:-} == --ros ]]; then
    shift
    unset WEBCAM_DDS_PORT
    exec "$@"
  fi
  if ! ffmpeg=$(webcam_ffmpeg_path) || [[ ! -x "$ffmpeg" ]]; then
    echo 'Run ./install_usb_webcam.sh first to install Windows FFmpeg.' >&2
    exit 1
  fi
  export WEBCAM_DDS_PORT=17677
  exec /usr/bin/python3 "$repo_dir/scripts/wsl_webcam_node.py" "$ffmpeg" "$@"
fi

if [[ ${1:-} == --ros ]]; then
  shift
  exec "$@"
fi
video_device=/dev/video0
if [[ ${1:-} == /dev/* ]]; then
  video_device=$1
  shift
fi
width=640
height=480
pixel_format=yuyv
case "${1:-}" in
  --480p) width=640; height=480; pixel_format=yuyv; shift ;;
  --720p) width=1280; height=720; pixel_format=mjpeg2rgb; shift ;;
  --1080p) width=1920; height=1080; pixel_format=mjpeg2rgb; shift ;;
esac
if [[ ! -c "$video_device" ]]; then
  printf 'Camera device unavailable: %s\n' "$video_device" >&2
  exit 1
fi
if [[ ! -r "$video_device" || ! -w "$video_device" ]]; then
  printf 'Camera device access denied: %s\n' "$video_device" >&2
  exit 1
fi
exec ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __node:=usb_webcam -r __ns:=/webcam \
  -p "video_device:=$video_device" -p camera_name:=usb_webcam \
  -p frame_id:=webcam_optical_frame -p "pixel_format:=$pixel_format" \
  -p "image_width:=$width" -p "image_height:=$height" -p framerate:=30.0 "$@"
