#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FAST-LIVO2 IMU-propagated odometry -> PX4 external vision.

Default input /LIVO2/imu_propagate is produced by the patched simulator launch.
Pose uses gravity-aligned local axes and the mounted IMU child frame. Twist is
in that IMU frame with positive linear covariance; legacy unknown twist stays
NaN. Simulation timestamps are translated through paired physics/PX4 IMU clock
mapping, without substituting publication/receive time for measurement time.
The ZAV simulator's +22 degree IMU mount and lever arm are removed for pose.
Epoch-stamped inputs retain the legacy body-aligned contract.

Dry-run by default. --publish --assume-enu-flu enables publication.
"""

import argparse
import fcntl
import os
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, SensorCombined, TimesyncStatus
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def stamp_us(stamp):
    return int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1000


def mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ]


def enu_to_ned(x, y, z):
    return [float(y), float(x), -float(z)]


def quat_enu_flu_to_ned_frd(orientation):
    q = [float(orientation.w), float(orientation.x),
         float(orientation.y), float(orientation.z)]
    norm = math.sqrt(sum(v*v for v in q))
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError("Invalid LIO quaternion")
    q = [v/norm for v in q]
    h = math.sqrt(0.5)
    result = mul(mul([0.0, h, h, 0.0], q), [0.0, 1.0, 0.0, 0.0])
    return [-v for v in result] if result[0] < 0.0 else result


def rotate(q, vector):
    return mul(mul(q, [0.0] + list(vector)), [q[0], -q[1], -q[2], -q[3]])[1:]


def diag_variances(covariance, first_indices, fallback):
    """Use diagonal variance when positive, else conservative fallback."""
    return [
        max(float(covariance[i]), fallback) if
        math.isfinite(float(covariance[i])) and float(covariance[i]) > 0.0
        else fallback
        for i in first_indices
    ]


class LioPx4Bridge(Node):
    def __init__(self, args):
        super().__init__("zav_lio_px4_odom_bridge")
        self.args = args
        self.pub = None
        self.timesync_offset = None
        self.sim_imu_us = 0
        self.sim_imu_mono_us = 0
        # ZAV simulator mount: IMU aligned to the Mid360 tilted +22 deg.
        angle = math.radians(22.0) / 2
        self.mount_q = [math.cos(angle), 0.0, math.sin(angle), 0.0]
        self.mount_position = [0.01215 + math.cos(2*angle)*0.011 + math.sin(2*angle)*0.04412,
                               0.02329,
                               0.11503 - math.sin(2*angle)*0.011 + math.cos(2*angle)*0.04412]
        self.latest_imu_stamp_ros_us = 0
        self.latest_imu_wall_us = 0
        self.latest_imu_mono_us = 0
        self.last_input_stamp = 0
        self.last_output_stamp = 0
        self.last_input_pos = None
        self.reset_counter = 0
        self.tokens = 1.0
        self.token_capacity = 2.0
        self.refill_mono_us = 0
        self.report_mono = time.monotonic()
        self.stats = dict(rx=0, imu=0, out=0, rate_drop=0,
                          stale_imu=0, stale_lio=0, no_imu=0,
                          nonmonotonic=0, jumps=0, bad_frame=0,
                          future=0, invalid=0)
        self.last_warning_mono = 0.0

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        px4_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, args.odom_topic,
                                 self.on_odom, qos)
        self.create_subscription(SensorCombined, "/fmu/out/sensor_combined",
                                 self.on_imu, qos)
        self.create_subscription(TimesyncStatus, "/fmu/out/timesync_status", self.on_timesync, qos)
        self.create_subscription(Imu, "/drone0/mid360/imu", self.on_sim_imu, qos)
        if args.publish:
            self.pub = self.create_publisher(
                VehicleOdometry, "/fmu/in/vehicle_visual_odometry", px4_qos)
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            f"mode={'PUBLISH' if args.publish else 'DRY-RUN'}; "
            f"input={args.odom_topic} output=/fmu/in/vehicle_visual_odometry; "
            f"limit={args.hz:g}Hz; velocity=validated twist or NaN; "
            f"pose_frame=FRD(local arbitrary heading)"
        )

    def warn(self, message):
        now = time.monotonic()
        if now - self.last_warning_mono > 2.0:
            self.get_logger().warn(message)
            self.last_warning_mono = now

    def on_timesync(self, message):
        self.timesync_offset = int(message.estimated_offset)

    def on_sim_imu(self, message):
        self.sim_imu_us = stamp_us(message.header.stamp)
        self.sim_imu_mono_us = time.monotonic_ns() // 1000

    def on_imu(self, message):
        timestamp = int(message.timestamp)
        wall_us = time.time_ns() // 1000
        if timestamp < 1_000_000_000_000:
            self.warn("PX4 IMU timestamp is not ROS-epoch time; check DDS timesync.")
            return
        if timestamp <= self.latest_imu_stamp_ros_us:
            return
        self.latest_imu_stamp_ros_us = timestamp
        self.latest_imu_wall_us = wall_us
        self.latest_imu_mono_us = time.monotonic_ns() // 1000
        self.stats["imu"] += 1

    def on_odom(self, odom):
        self.stats["rx"] += 1
        if odom.header.frame_id != "camera_init" or odom.child_frame_id != "aft_mapped":
            self.stats["bad_frame"] += 1
            self.warn("Unexpected frame_id / child_frame_id; blocking publication")
            return

        t_src = stamp_us(odom.header.stamp)
        if t_src <= self.last_input_stamp:
            self.stats["nonmonotonic"] += 1
            return
        self.last_input_stamp = t_src
        wall_us = time.time_ns() // 1000
        mono_us = time.monotonic_ns() // 1000
        simulation_clock = t_src < 1_000_000_000_000
        if simulation_clock:
            if self.timesync_offset is None or not self.sim_imu_us:
                self.stats["no_imu"] += 1
                return
            if mono_us - self.sim_imu_mono_us > self.args.max_imu_age_ms * 1000:
                self.stats["stale_imu"] += 1
                return
            age = self.sim_imu_us - t_src
        else:
            age = wall_us - t_src
        if not (0 <= age <= self.args.max_lio_age_ms * 1000):
            self.stats["stale_lio"] += 1
            self.warn(f"LIO time invalid/stale: reference minus header = {age/1000:.1f}ms")
            return

        if self.latest_imu_stamp_ros_us == 0:
            self.stats["no_imu"] += 1
            return
        imu_age = mono_us - self.latest_imu_mono_us
        if not (0 <= imu_age <= self.args.max_imu_age_ms * 1000):
            self.stats["stale_imu"] += 1
            self.warn(f"PX4 IMU time reference stale: {imu_age/1000:.1f}ms")
            return

        # Both simulated IMUs use the same physics clock. Pair their latest
        # samples to recover the current DDS clock mapping, preserving the LIO
        # measurement age. TimesyncStatus is lower rate and can lag the mapping
        # by hundreds of ms when simulation runs slower than wall time.
        # Epoch input keeps the legacy receive-time offset estimate.
        offset = (self.latest_imu_stamp_ros_us - self.sim_imu_us if simulation_clock else
                  self.latest_imu_stamp_ros_us - self.latest_imu_wall_us)
        corrected_sample = t_src + offset
        corrected_now = (self.sim_imu_us + offset if simulation_clock else wall_us + offset)
        if corrected_sample > corrected_now + 5000:
            self.stats["future"] += 1
            return
        if corrected_sample <= self.last_output_stamp:
            self.stats["nonmonotonic"] += 1
            return

        if self.refill_mono_us:
            dt_us = max(0, mono_us - self.refill_mono_us)
            self.tokens = min(
                self.token_capacity,
                self.tokens + dt_us * self.args.hz / 1e6
            )
        self.refill_mono_us = mono_us
        if self.tokens < 1.0:
            self.stats["rate_drop"] += 1
            return

        try:
            p = odom.pose.pose.position
            ori = odom.pose.pose.orientation
            imu_q = [ori.w, ori.x, ori.y, ori.z]
            body_q = mul(imu_q, [self.mount_q[0], -self.mount_q[1], -self.mount_q[2], -self.mount_q[3]]) if simulation_clock else imu_q
            lever = rotate(body_q, self.mount_position) if simulation_clock else [0., 0., 0.]
            body_orientation = type(ori)()
            body_orientation.w, body_orientation.x, body_orientation.y, body_orientation.z = body_q
            q = quat_enu_flu_to_ned_frd(body_orientation)
            position = enu_to_ned(p.x-lever[0], p.y-lever[1], p.z-lever[2])
            if not all(math.isfinite(v) for v in position + q):
                raise ValueError("Non-finite LIO pose")

            if self.last_input_pos is not None:
                d = math.dist(position, self.last_input_pos)
                if d > self.args.max_position_step_m:
                    self.stats["jumps"] += 1
                    self.warn(
                        f"LIO pose jumped {d:.3f}m in one message; "
                        "blocking this sample. Check estimator resets."
                    )
                    self.last_input_pos = position
                    return
            self.last_input_pos = position
        except ValueError as exc:
            self.stats["invalid"] += 1
            self.warn(str(exc))
            return

        v = VehicleOdometry()
        v.timestamp = int(corrected_now)
        v.timestamp_sample = int(corrected_sample)

        # camera_init is NOT known to be geographically aligned with NED.
        # Use a local arbitrary-heading frame rather than asserting true North.
        v.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        v.position = position
        v.q = q

        # Legacy FAST-LIVO has unimplemented all-zero twist. Keep it unknown
        # unless the publisher provides the patched positive-covariance contract.
        v.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        v.velocity = [math.nan, math.nan, math.nan]
        v.angular_velocity = [math.nan, math.nan, math.nan]

        cov = odom.pose.covariance
        v.position_variance = diag_variances(cov, (7, 0, 14), 0.04)
        v.orientation_variance = diag_variances(cov, (21, 28, 35), 0.03)
        v.velocity_variance = [1.0, 1.0, 1.0]
        if all(math.isfinite(odom.twist.covariance[i]) and odom.twist.covariance[i] > 0 for i in (0, 7, 14)):
            linear = odom.twist.twist.linear
            world_velocity = rotate(imu_q, [linear.x, linear.y, linear.z])
            if simulation_clock:
                angular = odom.twist.twist.angular
                omega = rotate(imu_q, [angular.x, angular.y, angular.z])
                lever_velocity = [omega[1]*lever[2]-omega[2]*lever[1],
                                  omega[2]*lever[0]-omega[0]*lever[2],
                                  omega[0]*lever[1]-omega[1]*lever[0]]
                world_velocity = [x-y for x,y in zip(world_velocity, lever_velocity)]
            if all(math.isfinite(x) for x in world_velocity):
                v.velocity_frame = VehicleOdometry.VELOCITY_FRAME_FRD
                v.velocity = enu_to_ned(*world_velocity)
                v.velocity_variance = [0.04, 0.04, 0.04]
        v.reset_counter = self.reset_counter
        v.quality = 100  # Placeholder: input provides no LIO quality score.

        if self.pub is not None:
            self.pub.publish(v)
        self.last_output_stamp = corrected_sample
        self.tokens -= 1.0
        self.stats["out"] += 1
        if self.stats["out"] == 1:
            self.get_logger().info(
                f"CHECK/TX source_age={age/1000:.1f}ms "
                f"imu_rx_age={imu_age/1000:.1f}ms "
                f"px4_clock_correction={offset/1000:.2f}ms "
                f"sample_stamp={corrected_sample} "
                f"pos_FRD={[round(x,4) for x in position]} "
                f"q_FRD={[round(x,4) for x in q]}"
            )

    def report(self):
        current = time.monotonic()
        elapsed = max(0.001, current - self.report_mono)
        self.report_mono = current
        values = dict(self.stats)
        self.stats = dict.fromkeys(self.stats, 0)
        self.get_logger().info(
            "STATS rx=%d imu=%d out=%.1fHz rate_drop=%d stale_lio=%d "
            "stale_imu=%d no_imu=%d bad_frame=%d jump=%d nonmonotonic=%d "
            "future=%d invalid=%d [%s]" %
            (values["rx"], values["imu"], values["out"] / elapsed,
             values["rate_drop"], values["stale_lio"], values["stale_imu"],
             values["no_imu"], values["bad_frame"], values["jumps"],
             values["nonmonotonic"], values["future"], values["invalid"],
             "PUBLISH" if self.pub is not None else "DRY-RUN")
        )


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--publish", action="store_true")
    parser.add_argument(
        "--assume-enu-flu", action="store_true",
        help="Explicitly confirm world Z-up/ENU-style and child body FLU axes, "
             "correct sensor/IMU mounting, before publishing"
    )
    parser.add_argument("--odom-topic", default="/LIVO2/imu_propagate")
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--max-lio-age-ms", type=float, default=250.0)
    parser.add_argument("--max-imu-age-ms", type=float, default=100.0)
    parser.add_argument("--max-position-step-m", type=float, default=1.0)
    ros_args = args
    args = parser.parse_args(remove_ros_args(args=ros_args)[1:])
    if args.publish and not args.assume_enu_flu:
        parser.error("--publish requires --assume-enu-flu after frame/extrinsic verification")
    if not (5 <= args.hz <= 100):
        parser.error("hz must be 5..100")
    if args.max_lio_age_ms <= 0 or args.max_imu_age_ms <= 0:
        parser.error("age limits must be positive")
    # One publisher owns the EV stream, including legacy Python invocation.
    publisher_lock = None
    if args.publish:
        publisher_lock = open(f"/tmp/zav-lio-px4-bridge-{os.getuid()}.lock", "w")
        try:
            fcntl.flock(publisher_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            publisher_lock.close()
            parser.error("LIO -> PX4 bridge already running; stop the other publisher first")
    rclpy.init(args=ros_args)
    node = LioPx4Bridge(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if publisher_lock is not None:
            publisher_lock.close()


if __name__ == "__main__":
    main()
