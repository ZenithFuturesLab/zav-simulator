"""[ZAV 변경] Isaac Python의 센서 스트림을 시스템 ROS 2 토픽으로 변환합니다.

Ubuntu/WSL 공통 수신기. Isaac Python 3.11과 Humble의 사용자 메시지 ABI를
분리하므로 livox_ros_driver2는 Isaac 내부에 설치할 필요가 없습니다.
"""
import argparse
import socket
import signal
import struct
import threading
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.serialization import deserialize_message
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Image, Imu

def read_exact(conn, count):
    chunks = bytearray()
    while len(chunks) < count:
        part = conn.recv(count - len(chunks))
        if not part:
            raise ConnectionError('TCP source disconnected')
        chunks.extend(part)
    return bytes(chunks)


def client(args):
    # Both FAST-LIVO and RViz may request the relay; publish each sensor once.
    import fcntl
    relay_lock = open('/tmp/zav-sensor-relay.lock', 'w')
    try:
        fcntl.flock(relay_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('Sensor relay already running.', flush=True)
        return
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('mid360_tcp_receiver')
    publisher = node.create_publisher(PointCloud2, args.topic, qos_profile_sensor_data)
    count = 0
    started = time.monotonic()
    camera_stop = threading.Event()
    camera_worker = None
    imu_worker = None
    livox_worker = None
    if args.livox_port:
        import numpy as np
        from livox_ros_driver2.msg import CustomMsg, CustomPoint
        livox_pub = node.create_publisher(CustomMsg, args.livox_topic, QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
        point_dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('offset', '<u4'),
                                ('reflectivity', 'u1'), ('line', 'u1'), ('tag', 'u1'), ('reserved', 'u1')])

        def receive_livox():
            scans = 0
            while not camera_stop.is_set() and rclpy.ok():
                try:
                    with socket.create_connection((args.host, args.livox_port), timeout=3) as conn:
                        conn.settimeout(2)
                        while not camera_stop.is_set() and rclpy.ok():
                            size = struct.unpack('!I', read_exact(conn, 4))[0]
                            if not 12 <= size <= 64 * 1024 * 1024:
                                raise ValueError(f'Invalid Livox frame size {size}')
                            payload = read_exact(conn, size)
                            stamp_ns, count = struct.unpack_from('<QI', payload)
                            if size != 12 + count * point_dtype.itemsize:
                                raise ValueError('Livox payload length mismatch')
                            points = np.frombuffer(payload, dtype=point_dtype, offset=12)
                            msg = CustomMsg()
                            msg.header.frame_id = 'mid360'
                            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 10**9)
                            msg.timebase = stamp_ns
                            msg.point_num = count
                            msg.points = [CustomPoint(x=float(p['x']), y=float(p['y']), z=float(p['z']),
                                                      offset_time=int(p['offset']), reflectivity=int(p['reflectivity']),
                                                      line=int(p['line']), tag=int(p['tag'])) for p in points]
                            livox_pub.publish(msg)
                            scans += 1
                            if scans == 1 or scans % 100 == 0:
                                print(f'LIVOX_SCANS={scans} points={count} timebase={stamp_ns} last_offset_ns={int(points["offset"][-1]) if count else 0}', flush=True)
                except (OSError, ConnectionError, ValueError) as exc:
                    if not camera_stop.is_set():
                        print(f'LIVOX_RETRY {exc}', flush=True)
                        camera_stop.wait(1)

        livox_worker = threading.Thread(target=receive_livox, daemon=True)
        livox_worker.start()
    if args.imu_port:
        imu_pub = node.create_publisher(Imu, args.imu_topic, QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT))

        def receive_imu():
            samples = 0
            while not camera_stop.is_set() and rclpy.ok():
                try:
                    with socket.create_connection((args.host, args.imu_port), timeout=3) as conn:
                        conn.settimeout(2)
                        while not camera_stop.is_set() and rclpy.ok():
                            size = struct.unpack('!I', read_exact(conn, 4))[0]
                            if not 0 < size <= 64 * 1024:
                                raise ValueError(f'Invalid IMU frame size {size}')
                            msg = deserialize_message(read_exact(conn, size), Imu)
                            imu_pub.publish(msg)
                            samples += 1
                            if samples == 1 or samples % 2500 == 0:
                                print(f'IMU_SAMPLES={samples} frame={msg.header.frame_id}', flush=True)
                except (OSError, ConnectionError, ValueError) as exc:
                    if not camera_stop.is_set():
                        print(f'IMU_RETRY {exc}', flush=True)
                        camera_stop.wait(1)

        imu_worker = threading.Thread(target=receive_imu, daemon=True)
        imu_worker.start()
    if args.camera_port:
        image_pub = node.create_publisher(Image, args.camera_topic, qos_profile_sensor_data)

        def receive_images():
            frames = 0
            while not camera_stop.is_set() and rclpy.ok():
                try:
                    with socket.create_connection((args.host, args.camera_port), timeout=3) as conn:
                        conn.settimeout(2)
                        while not camera_stop.is_set() and rclpy.ok():
                            size = struct.unpack('!I', read_exact(conn, 4))[0]
                            if not 0 < size <= 64 * 1024 * 1024:
                                raise ValueError(f'Invalid image frame size {size}')
                            msg = deserialize_message(read_exact(conn, size), Image)
                            if len(msg.data) != msg.height * msg.step:
                                raise ValueError('Image payload size mismatch')
                            image_pub.publish(msg)
                            frames += 1
                            if frames == 1 or frames % 300 == 0:
                                print(f'CAMERA_FRAMES={frames} size={msg.width}x{msg.height} encoding={msg.encoding}', flush=True)
                except (OSError, ConnectionError, ValueError) as exc:
                    if not camera_stop.is_set():
                        print(f'CAMERA_RETRY {exc}', flush=True)
                        camera_stop.wait(1)

        camera_worker = threading.Thread(target=receive_images, daemon=True)
        camera_worker.start()
    try:
        while rclpy.ok():
            try:
                with socket.create_connection((args.host, args.port), timeout=3) as conn:
                    conn.settimeout(5)
                    print(f'CLIENT_CONNECTED {args.host}:{args.port}', flush=True)
                    while rclpy.ok():
                        size = struct.unpack('!I', read_exact(conn, 4))[0]
                        if not 0 < size <= 64 * 1024 * 1024:
                            raise ValueError(f'Invalid frame size {size}')
                        msg = deserialize_message(read_exact(conn, size), PointCloud2)
                        publisher.publish(msg)
                        count += 1
                        if count == 1 or count % 300 == 0:
                            print(f'RELAY_MESSAGES={count} points={msg.width * msg.height} bytes={len(msg.data)} frame={msg.header.frame_id}', flush=True)
                        rclpy.spin_once(node, timeout_sec=0)
                        if args.seconds and time.monotonic() - started >= args.seconds:
                            return
            except (OSError, ConnectionError, ValueError) as exc:
                print(f'RETRY {exc}', flush=True)
                if args.seconds and time.monotonic() - started >= args.seconds:
                    return
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        camera_stop.set()
        if camera_worker is not None:
            camera_worker.join(timeout=5)
        if imu_worker is not None:
            imu_worker.join(timeout=5)
        if livox_worker is not None:
            livox_worker.join(timeout=5)
        print(f'CLIENT_RESULT messages={count}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    def stop_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_signal)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['client'])
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=17441)
    parser.add_argument('--topic', default='/drone0/mid360/pointcloud')
    parser.add_argument('--seconds', type=float, default=0)
    parser.add_argument('--camera-port', type=int)
    parser.add_argument('--camera-topic', default='/drone0/front_camera/color/image_raw')
    parser.add_argument('--imu-port', type=int)
    parser.add_argument('--imu-topic', default='/drone0/mid360/imu')
    parser.add_argument('--livox-port', type=int)
    parser.add_argument('--livox-topic', default='/drone0/mid360/lidar')
    args = parser.parse_args()
    client(args)
