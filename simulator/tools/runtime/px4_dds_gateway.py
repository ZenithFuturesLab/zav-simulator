# [ZAV 변경] WSL 전용 Fast DDS ↔ Cyclone TCP 경계. native Ubuntu는 이 모듈을 실행하지 않습니다.
"""Bridge local PX4 Fast DDS endpoints to the WSL Cyclone TCP sensor graph.

Separate processes are required: an rclpy context cannot switch RMW libraries.
Only the explicit PX4 telemetry/control routes below cross the local Pipe.
"""
import fcntl
import multiprocessing as mp
import os
from pathlib import Path
import signal
import time

ROUTES = [
    ("SensorCombined", "/fmu/out/sensor_combined", "fast"),
    ("TimesyncStatus", "/fmu/out/timesync_status", "fast"),
    ("VehicleOdometry", "/fmu/in/vehicle_visual_odometry", "cyclone"),
    ("VehicleStatus", "/fmu/out/vehicle_status", "fast"),
    ("VehicleLocalPosition", "/fmu/out/vehicle_local_position", "fast"),
    ("VehicleOdometry", "/fmu/out/vehicle_odometry", "fast"),
    ("VehicleCommandAck", "/fmu/out/vehicle_command_ack", "fast"),
    ("OffboardControlMode", "/fmu/in/offboard_control_mode", "cyclone"),
    ("TrajectorySetpoint", "/fmu/in/trajectory_setpoint", "cyclone"),
    ("VehicleCommand", "/fmu/in/vehicle_command", "cyclone"),
]


def worker(side, connection, stop):
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp" if side == "fast" else "rmw_cyclonedds_cpp"
    for key in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
        os.environ.pop(key, None)
    if side == "cyclone":
        os.environ["CYCLONEDDS_URI"] = (Path(__file__).resolve().parents[1] / "rviz/cyclonedds_px4.xml").as_uri()
    else:
        os.environ.pop("CYCLONEDDS_URI", None)
    import rclpy
    import px4_msgs.msg as messages
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import serialize_message, deserialize_message
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("px4_gateway_" + side)
    # Preserve the PX4 transient-local contract required by flight_manager.
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    incoming_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
    publishers = {}
    subscriptions = []
    counts = {}
    def callback(index):
        def receive(msg):
            connection.send_bytes(bytes([index]) + serialize_message(msg))
            counts[index] = counts.get(index, 0) + 1
            if counts[index] == 1:
                print(f"GATEWAY {side} first {ROUTES[index][1]}", flush=True)
        return receive
    for index, (typename, topic, source) in enumerate(ROUTES):
        typ = getattr(messages, typename)
        if source == side:
            subscriptions.append(node.create_subscription(typ, topic, callback(index), incoming_qos))
        else:
            publishers[index] = (node.create_publisher(typ, topic, qos), typ)
    print(f"GATEWAY_READY {side}", flush=True)
    try:
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.005)
            for _ in range(100):
                if not connection.poll():
                    break
                payload = connection.recv_bytes()
                publisher, typ = publishers[payload[0]]
                publisher.publish(deserialize_message(payload[1:], typ))
    except (EOFError, BrokenPipeError, KeyboardInterrupt):
        pass
    finally:
        stop.set()
        node.destroy_node()
        rclpy.shutdown()
        connection.close()


def main():
    # Cyclone TCP permits address reuse; an OS listener alone is not a singleton.
    lock = open(f"/tmp/zav-px4-gateway-{os.getuid()}.lock", "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("PX4 DDS gateway is already running", flush=True)
        return 1
    context = mp.get_context("spawn")
    stop = context.Event()
    left, right = context.Pipe(duplex=True)
    children = [context.Process(target=worker, args=("fast", left, stop)),
                context.Process(target=worker, args=("cyclone", right, stop))]
    stopping = False
    def halt(signum, frame):
        # Never acquire multiprocessing.Event locks from a signal handler.
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, halt)
    signal.signal(signal.SIGTERM, halt)
    for child in children:
        child.start()
    left.close()
    right.close()
    try:
        while not stopping and not stop.is_set():
            if not all(child.is_alive() for child in children):
                break
            time.sleep(.2)
    finally:
        stop.set()
        for child in children:
            child.join(timeout=3)
            if child.is_alive():
                child.terminate()
                child.join(timeout=2)
                if child.is_alive():
                    child.kill()
                    child.join(timeout=2)
    return max(child.exitcode or 0 for child in children)


if __name__ == "__main__":
    raise SystemExit(main())
