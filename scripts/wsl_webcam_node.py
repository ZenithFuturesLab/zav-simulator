"""Publish a Windows DirectShow camera from WSL over local ROS 2 topics."""

import argparse
from array import array
import re
import select
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

PRESETS = {
    "480p": (640, 480, 30, None),
    "720p": (1280, 720, 60, "mjpeg"),
    "1080p": (1920, 1080, 30, "mjpeg"),
}


def find_camera(ffmpeg):
    result = subprocess.run(
        [ffmpeg, "-hide_banner", "-list_devices", "true", "-f", "dshow", "-i", "dummy"],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        encoding="utf-8",
        errors="replace",
        timeout=10,
        check=False,
    )
    # Device enumeration exits after printing the list, with no input opened.
    for line in result.stderr.splitlines():
        match = re.search(r'"(.+)"\s+\(video\)', line)
        if match:
            return match.group(1)
    raise RuntimeError("No Windows video device found. Check camera access in Windows.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ffmpeg")
    parser.add_argument("--camera", help="Windows DirectShow device name")
    preset_group = parser.add_mutually_exclusive_group()
    for preset, (width, height, _, _) in PRESETS.items():
        preset_group.add_argument(f"--{preset}", dest="preset", action="store_const",
                                  const=preset, help=f"publish {width}x{height} at 30 FPS")
    args = parser.parse_args()
    width, height, capture_fps, codec = PRESETS[args.preset or "480p"]
    step = width * 3
    frame_bytes = step * height
    input_codec = ["-vcodec", codec] if codec else []
    output_rate = ["-vf", "fps=30"] if capture_fps != 30 else []

    rclpy.init(args=[])
    node = Node("usb_webcam", namespace="/webcam")
    image_pub = node.create_publisher(Image, "image_raw", 1)
    info_pub = node.create_publisher(CameraInfo, "camera_info", qos_profile_sensor_data)
    camera = None
    try:
        camera_name = args.camera or find_camera(args.ffmpeg)
        node.get_logger().info(
            f"Opening Windows camera {camera_name!r} at {width}x{height}, "
            f"capture {capture_fps} FPS, publish 30 FPS"
        )
        camera = subprocess.Popen(
            [
                args.ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin",
                "-f", "dshow", "-video_size", f"{width}x{height}",
                "-framerate", str(capture_fps), *input_codec,
                "-i", f"video={camera_name}", "-an", *output_rate, "-pix_fmt", "rgb24",
                "-f", "rawvideo", "pipe:1",
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            bufsize=0,
            start_new_session=True,
        )
        frame = bytearray()
        while rclpy.ok():
            ready, _, _ = select.select([camera.stdout], [], [], 5)
            if not ready:
                raise RuntimeError("Timed out waiting for a camera frame")
            chunk = camera.stdout.read(frame_bytes - len(frame))
            if not chunk:
                raise RuntimeError("FFmpeg stopped sending camera frames")
            frame.extend(chunk)
            if len(frame) != frame_bytes:
                continue

            image = Image(width=width, height=height, encoding="rgb8", step=step)
            image.header.stamp = node.get_clock().now().to_msg()
            image.header.frame_id = "webcam_optical_frame"
            image.data = array("B", frame)
            image_pub.publish(image)
            # Intrinsics stay zero until a camera calibration is provided.
            info_pub.publish(CameraInfo(header=image.header, width=width, height=height))
            frame.clear()
            rclpy.spin_once(node, timeout_sec=0)
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        if rclpy.ok():
            node.get_logger().error(str(exc))
            return 1
    finally:
        if camera is not None:
            camera.stdout.close()
            if camera.poll() is None:
                camera.terminate()
                try:
                    camera.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    camera.kill()
                    camera.wait()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
