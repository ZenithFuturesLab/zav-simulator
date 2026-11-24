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

WIDTH = 640
HEIGHT = 480
STEP = WIDTH * 3
FRAME_BYTES = STEP * HEIGHT


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
    args = parser.parse_args()

    rclpy.init(args=[])
    node = Node("usb_webcam", namespace="/webcam")
    image_pub = node.create_publisher(Image, "image_raw", 1)
    info_pub = node.create_publisher(CameraInfo, "camera_info", qos_profile_sensor_data)
    camera = None
    try:
        camera_name = args.camera or find_camera(args.ffmpeg)
        node.get_logger().info(f"Opening Windows camera {camera_name!r} at {WIDTH}x{HEIGHT}, 30 FPS")
        camera = subprocess.Popen(
            [
                args.ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin",
                "-f", "dshow", "-video_size", f"{WIDTH}x{HEIGHT}", "-framerate", "30",
                "-i", f"video={camera_name}", "-an", "-pix_fmt", "rgb24",
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
            chunk = camera.stdout.read(FRAME_BYTES - len(frame))
            if not chunk:
                raise RuntimeError("FFmpeg stopped sending camera frames")
            frame.extend(chunk)
            if len(frame) != FRAME_BYTES:
                continue

            image = Image(width=WIDTH, height=HEIGHT, encoding="rgb8", step=STEP)
            image.header.stamp = node.get_clock().now().to_msg()
            image.header.frame_id = "webcam_optical_frame"
            image.data = array("B", frame)
            image_pub.publish(image)
            # Intrinsics stay zero until a camera calibration is provided.
            info_pub.publish(CameraInfo(header=image.header, width=WIDTH, height=HEIGHT))
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
