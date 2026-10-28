#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
    ZAV Flight Manager (Bridge Node & Safety Gateway)
==============================================================================
기체: 26 SfUAV ZAV 커스텀 쿼드콥터 (1.99 kg, T-Motor F90 6S)

[브릿지 노드 3대 핵심 역할]
1. Manual Position Teleop:
   - 기체 로컬 프레임(Body Frame) 기준 전/후/좌/우/상/하 비행
   - 전진 중 Yaw 회전 시 기수가 돌아가는 방향을 따라 실시간 선회(Heading Follow)
   - 부드러운 Yaw 회전 튜닝 (1.0 rad/s 선형 가감속)
   - 키를 놓으면 즉시 제동 및 제자리 호버링
   - 좌우 방향키 반대 현상 보정 (←: 왼쪽, →: 오른쪽)

2. Offboard Mode (상위 플래너 연동):
   - 상위 자율비행 플래너 목표 위치(/zav/goal_pose) 및 속도(/zav/cmd_vel) 수신
   - 목표 위치 수신 시 PX4 TrajectorySetpoint의 3차원 위치 명령으로 자율 이동
   - 자율비행 중에는 수동 키보드 이동 입력을 안전하게 차단(Safety Gate)
   - 숫자키 1: Manual 모드 / 숫자키 2: Offboard 자율비행 모드

3. PX4 <-> ROS 2 좌표계 변환:
   - 목표 위치 좌표계: X(North), Y(East), Z(Up)
   - PX4 관성 항법 프레임: Inertial NED (North: +X, East: +Y, Down: +Z)
   - 기체의 실시간 헤딩 psi(Yaw)를 기반으로 속도 및 위치 벡터를 정확히 변환
==============================================================================
"""

import os
import sys
import base64
import time
import math
import select
import termios
import tty
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# ROS 2 표준 메시지
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool

# PX4 uORB 메시지 인터페이스
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)

# 비트마스크 상수 정의
KEY_UP    = 1 << 0  # 1   (전진 ↑)
KEY_DOWN  = 1 << 1  # 2   (후진 ↓)
KEY_LEFT  = 1 << 2  # 4   (좌측 ←)
KEY_RIGHT = 1 << 3  # 8   (우측 →)
KEY_W     = 1 << 4  # 16  (상승 W)
KEY_S     = 1 << 5  # 32  (하강 S)
KEY_A     = 1 << 6  # 64  (좌회전 A)
KEY_D     = 1 << 7  # 128 (우회전 D)
KEY_SPACE = 1 << 8  # 256 (긴급 정지 Spacebar)


def wrap_pi(theta: float) -> float:
    """각도를 [-pi, +pi] 범위로 정규화"""
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """쿼터니언을 Yaw(라디안)로 변환"""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-4:
        return float('nan')
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def ramp_value(current: float, target: float, accel_rate: float, decel_rate: float, dt: float) -> float:
    """
    [스틱 선형 가감속 함수 (Linear Ramp / Slew-Rate Limiter)]
    - 스틱을 밀 때(가속)는 accel_rate로 선형 증가
    - 스틱을 놓거나 반대 방향 꺾을 때(감속)는 decel_rate로 즉각 복귀
    """
    if abs(target) < 1e-5 or (current * target < 0.0):
        rate = decel_rate
    else:
        rate = accel_rate

    step = rate * dt
    if target > current:
        return min(target, current + step)
    elif target < current:
        return max(target, current - step)
    return target


class Win32MultiKeyReader:
    """
    WSL2 환경에서 Windows user32.dll GetAsyncKeyState를 호출하여
    버퍼 지연 없이 최신 키 눌림 상태(비트마스크)를 읽어오는 리더
    """
    def __init__(self):
        self.key_mask = 0
        self.active = False
        self._proc = None
        self._pipe_fd = None
        self._recv_buf = b""

        ps_exe = "/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"
        if os.environ.get("ZAV_SIM_PLATFORM") != "native" and os.path.exists(ps_exe):
            ps_code = (
                "Add-Type -TypeDefinition '"
                "using System; using System.Runtime.InteropServices; "
                "public class KS { [DllImport(\"user32.dll\")] public static extern short GetAsyncKeyState(int vKey); }"
                "'; "
                "while ($true) { "
                "$m = 0; "
                "if (([int][KS]::GetAsyncKeyState(0x26) -band 0x8000) -ne 0) { $m = $m -bor 1 }; "
                "if (([int][KS]::GetAsyncKeyState(0x28) -band 0x8000) -ne 0) { $m = $m -bor 2 }; "
                "if (([int][KS]::GetAsyncKeyState(0x25) -band 0x8000) -ne 0) { $m = $m -bor 4 }; "
                "if (([int][KS]::GetAsyncKeyState(0x27) -band 0x8000) -ne 0) { $m = $m -bor 8 }; "
                "if (([int][KS]::GetAsyncKeyState(0x57) -band 0x8000) -ne 0) { $m = $m -bor 16 }; "
                "if (([int][KS]::GetAsyncKeyState(0x53) -band 0x8000) -ne 0) { $m = $m -bor 32 }; "
                "if (([int][KS]::GetAsyncKeyState(0x41) -band 0x8000) -ne 0) { $m = $m -bor 64 }; "
                "if (([int][KS]::GetAsyncKeyState(0x44) -band 0x8000) -ne 0) { $m = $m -bor 128 }; "
                "if (([int][KS]::GetAsyncKeyState(0x20) -band 0x8000) -ne 0) { $m = $m -bor 256 }; "
                "[Console]::Out.WriteLine($m); "
                "[Console]::Out.Flush(); "
                "Start-Sleep -Milliseconds 20 "
                "}"
            )
            encoded = base64.b64encode(ps_code.encode('utf-16le')).decode('ascii')
            try:
                self._proc = subprocess.Popen(
                    [ps_exe, "-NoProfile", "-NonInteractive", "-EncodedCommand", encoded],
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    bufsize=0
                )
                self._pipe_fd = self._proc.stdout.fileno()
                os.set_blocking(self._pipe_fd, False)
            except Exception:
                self.active = False

    def poll_latest(self):
        """파이프에 쌓인 버퍼를 모두 비우고 최신 키 상태만 추출"""
        if self._pipe_fd is None:
            return 0
        try:
            while True:
                data = os.read(self._pipe_fd, 4096)
                if not data:
                    break
                self._recv_buf += data
        except BlockingIOError:
            pass
        except Exception:
            return self.key_mask

        if b"\n" in self._recv_buf:
            lines = self._recv_buf.split(b"\n")
            self._recv_buf = lines[-1]
            for raw_line in reversed(lines[:-1]):
                s = raw_line.strip()
                if s.isdigit():
                    self.key_mask = int(s)
                    self.active = True
                    break
        return self.key_mask

    def stop(self):
        if self._proc:
            try:
                self._proc.kill()
            except Exception:
                pass


def read_key_nonblocking(fd: int) -> str:
    """리눅스 터미널 비차단 단일 키 입력"""
    rlist, _, _ = select.select([fd], [], [], 0.005)
    if not rlist:
        return ''
    try:
        raw_bytes = os.read(fd, 32)
        return raw_bytes.decode('utf-8', errors='ignore')
    except Exception:
        return ''


class ZAVFlightManager(Node):
    def __init__(self):
        super().__init__('zav_flight_manager')

        # ----------------------------------------------------------------------
        # 1. 상태 및 비행 파라미터
        # ----------------------------------------------------------------------
        self.is_armed = False
        self.nav_state = 0
        self.is_flying = False
        self.is_taking_off = False
        # [ZAV 변경] 수동 정지 위치와 자동 착륙 중 외부 setpoint 차단 상태입니다.
        self.manual_hold = None
        self.manual_z_hold = None
        self.setpoints_enabled = True
        self.lio_safety_latched = False
        self.lio_safety_ready = None
        self.lio_safety_rx = None

        # 비행 모드 (1: MANUAL, 2: OFFBOARD_TRAJ)
        self.flight_mode = "MANUAL"

        # 수평 속도 파라미터 (정밀 정지용 기본 1.5 m/s)
        self.max_speed_xy = float(self.declare_parameter('manual_speed_xy', 12).value)
        self.accel_xy = float(self.declare_parameter('manual_accel_xy', 5.0).value)
        self.decel_xy = float(self.declare_parameter('manual_decel_xy', 14.0).value)

        # 수직 속도 파라미터 (상승 3.0 m/s, 하강 1.5 m/s)
        self.max_speed_z_up = 3.0
        self.max_speed_z_dn = 1.5
        self.accel_z = 2.5
        self.decel_z = 6.0

        # [튜닝 완료] 부드러운 Yaw 회전 파라미터 (최대 1.0 rad/s ≈ 57.3°/s)
        self.max_yaw_rate = 1.0
        self.accel_yaw = 1.2
        self.decel_yaw = 2.5

        # 센서 피드백
        self.current_alt = 0.0
        self.current_speed = float('inf')
        self.brake_settled_since = None
        self.position_ready = False
        self.position_announced = False
        self.last_position_time = 0.0
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.takeoff_x = 0.0
        self.takeoff_y = 0.0
        self.yaw_initialized = False

        # 안전 호버링용 저장 좌표
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z = -2.0
        self.hold_yaw = 0.0

        # 목표 스틱값(des_*) 및 선형 보간된 현재 명령값(cmd_*)
        self.des_vx_body = 0.0
        self.des_vy_body = 0.0
        self.des_vz_ned = 0.0
        self.des_yaw_rate = 0.0

        self.cmd_vx_body = 0.0
        self.cmd_vy_body = 0.0
        self.cmd_vz_ned = 0.0
        self.cmd_yaw_rate = 0.0

        self.target_yaw = 0.0
        self.was_turning_yaw = False
        self.was_moving = False
        self.last_print_time = 0.0

        # 상위 플래너 외부 명령 버퍼
        # 1) 목표 위치(/zav/goal_pose): [x, y, z, yaw]
        self.ext_goal_pose = None
        self.last_goal_time = 0.0

        # 2) 목표 속도(/zav/cmd_vel): Twist
        self.ext_cmd_vel = None
        self.last_ext_vel_time = 0.0

        # 무지연 다중 키 리더 시작
        self.multi_key = Win32MultiKeyReader()

        # 리눅스 터미널 폴백용 릴리즈 타이머
        self.vx_release_time = 0.0
        self.vy_release_time = 0.0
        self.vz_release_time = 0.0
        self.yaw_release_time = 0.0

        # ----------------------------------------------------------------------
        # 2. 송신 QoS: BEST_EFFORT + TRANSIENT_LOCAL (수신은 아래에서 별도 설정)
        # ----------------------------------------------------------------------
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # PX4 uORB 퍼블리셔
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_px4)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_px4)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_px4)

        # [ZAV 변경] 수신은 VOLATILE: Ubuntu의 PX4 직접 연결과 WSL 게이트웨이 양쪽을 수용합니다.
        # TRANSIENT_LOCAL 구독은 VOLATILE publisher와 연결되지 않습니다.
        qos_feedback = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                  durability=DurabilityPolicy.VOLATILE)
        # [ZAV 변경] 고정된 PX4 v1.14.3/px4_msgs의 실제 토픽만 구독합니다. 이름만 _v1/_v4로 바꿔도 ABI는 호환되지 않습니다.
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self._status_cb, qos_feedback)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._pos_cb, qos_feedback)
        safety_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/zav/lio_safety/land_requested',
                                 self._lio_safety_cb, safety_qos)
        self.create_subscription(Bool, '/zav/lio_safety/ready',
                                 self._lio_ready_cb, safety_qos)

        # 상위 플래너 외부 입력 인터페이스
        # 1. 위치 목표 (Position Command / Goal Pose)
        self.create_subscription(PoseStamped, '/zav/goal_pose', self._on_goal_pose, 10)
        # 2. 속도 목표 (Velocity Command)
        self.create_subscription(Twist, '/zav/cmd_vel', self._on_cmd_vel, 10)

        # 50Hz (20ms) 고정밀 제어 루프
        self.dt = 0.02
        self.last_loop_time = time.time()
        self.timer = self.create_timer(self.dt, self._control_loop)

        self._print_guide()

    # --------------------------------------------------------------------------
    # 센서 및 상태 피드백 콜백
    # --------------------------------------------------------------------------
    def _lio_ready_cb(self, msg):
        self.lio_safety_ready = bool(msg.data)
        self.lio_safety_rx = time.monotonic()

    def _lio_safety_cb(self, msg):
        if not msg.data or self.lio_safety_latched:
            return
        self.lio_safety_latched = True
        self.setpoints_enabled = False
        self.is_flying = self.is_taking_off = False
        self.ext_goal_pose = self.ext_cmd_vel = None
        self.des_vx_body = self.cmd_vx_body = 0.0
        self.des_vy_body = self.cmd_vy_body = 0.0
        self.des_vz_ned = self.cmd_vz_ned = 0.0
        self.des_yaw_rate = self.cmd_yaw_rate = 0.0
        print('\r\n[LIO FAILSAFE] 착륙 요청: 외부 setpoint와 재이륙 차단. 지상에서 노드 재시작 필요.\r\n')

    def _status_cb(self, msg: VehicleStatus):
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state

    def _pos_cb(self, msg: VehicleLocalPosition):
        """실시간 고도 및 헤딩 갱신 (선회 비행의 핵심)"""
        # [ZAV 변경] 실제 위치 추정 유효성/수신 시각을 기준으로 이륙 준비를 판단합니다.
        self.position_ready = bool(msg.xy_valid and msg.z_valid)
        self.last_position_time = time.monotonic()
        if self.position_ready and not self.position_announced:
            self.position_announced = True
            print("\r\n[PX4] 유효한 위치 데이터 수신 완료. T 키로 이륙할 수 있습니다.\r\n")
        self.current_alt = -msg.z
        self.current_speed = math.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2) if msg.v_xy_valid and msg.v_z_valid else float('inf')
        self.current_yaw = msg.heading
        self.current_x = msg.x
        self.current_y = msg.y
        if not self.yaw_initialized:
            self.target_yaw = wrap_pi(msg.heading)
            self.yaw_initialized = True
        if self.setpoints_enabled and self.current_alt > 0.5 and not self.is_taking_off and not self.is_flying:
            self.is_flying = True
            self.target_yaw = wrap_pi(msg.heading)

    def _on_goal_pose(self, msg: PoseStamped):
        """
        [상위 플래너 위치 목표 콜백]
        목표 좌표계: X(North), Y(East), Z(Up). camera_init와 동일하지 않습니다.
        """
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z
        ori = msg.pose.orientation
        yaw = quat_to_yaw(ori.x, ori.y, ori.z, ori.w)

        self.ext_goal_pose = (px, py, pz, yaw)
        self.last_goal_time = time.time()
        print(f"\r\n🎯 [목표 수신] Offboard 목표 위치 -> X:{px:.2f}m, Y:{py:.2f}m, Z:{pz:.2f}m\r\n")

    def _on_cmd_vel(self, msg: Twist):
        """[상위 플래너 속도 목표 콜백]"""
        self.ext_cmd_vel = msg
        self.last_ext_vel_time = time.time()

    def publish_offboard_mode(self):
        """PX4 Offboard 위치/속도 제어 모드 플래그 송신"""
        mode = OffboardControlMode()
        mode.position = True
        mode.velocity = True
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.timestamp = 0
        self.offboard_mode_pub.publish(mode)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """PX4 MAVLink 시스템 명령 전송"""
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.param7 = float(param7)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = 0
        self.cmd_pub.publish(cmd)

    # --------------------------------------------------------------------------
    # [핵심 1] Manual 입력을 받는 Position Teleop 처리
    # --------------------------------------------------------------------------
    def _update_multi_key_targets(self):
        mask = self.multi_key.key_mask

        # Offboard 자율비행 모드일 때는 수동 이동키 완전 차단 (안전 게이트웨이)
        if self.flight_mode == "OFFBOARD_TRAJ":
            self.des_vx_body = 0.0
            self.des_vy_body = 0.0
            self.des_vz_ned = 0.0
            self.des_yaw_rate = 0.0
            return

        # Spacebar: 긴급 제자리 브레이크 (모든 속도 0)
        if mask & KEY_SPACE:
            self.des_vx_body = self.cmd_vx_body = 0.0
            self.des_vy_body = self.cmd_vy_body = 0.0
            self.des_vz_ned = self.cmd_vz_ned = 0.0
            self.des_yaw_rate = self.cmd_yaw_rate = 0.0
            self.target_yaw = wrap_pi(self.current_yaw)
            return

        # 1) 전후 축 (Up / Down) -> 드론 머리 앞(+vx) / 뒤(-vx)
        if (mask & KEY_UP) and not (mask & KEY_DOWN):
            self.des_vx_body = self.max_speed_xy
        elif (mask & KEY_DOWN) and not (mask & KEY_UP):
            self.des_vx_body = -self.max_speed_xy
        else:
            self.des_vx_body = 0.0

        # 2) 좌우 축 (Left / Right) -> 좌우 반대 보정 완료
        # ← (KEY_LEFT)  : 드론 머리 기준 '왼쪽' (-vy_body)
        # → (KEY_RIGHT) : 드론 머리 기준 '오른쪽' (+vy_body)
        if (mask & KEY_LEFT) and not (mask & KEY_RIGHT):
            self.des_vy_body = -self.max_speed_xy
        elif (mask & KEY_RIGHT) and not (mask & KEY_LEFT):
            self.des_vy_body = self.max_speed_xy
        else:
            self.des_vy_body = 0.0

        # 3) 수직 축 (W / S) -> 상승(-vz) / 하강(+vz) (NED 기준)
        if (mask & KEY_W) and not (mask & KEY_S):
            self.des_vz_ned = -self.max_speed_z_up
            if self.is_armed and self.nav_state != 14:
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif (mask & KEY_S) and not (mask & KEY_W):
            self.des_vz_ned = self.max_speed_z_dn
        else:
            self.des_vz_ned = 0.0

        # 4) Yaw 회전 축 (A / D) -> 부드러운 좌회전(-rate) / 우회전(+rate)
        if (mask & KEY_A) and not (mask & KEY_D):
            self.des_yaw_rate = -self.max_yaw_rate
        elif (mask & KEY_D) and not (mask & KEY_A):
            self.des_yaw_rate = self.max_yaw_rate
        else:
            self.des_yaw_rate = 0.0

    # --------------------------------------------------------------------------
    # 50Hz 고정밀 메인 제어 루프
    # --------------------------------------------------------------------------
    def _control_loop(self):
        now = time.time()
        dt = min(0.1, max(0.005, now - self.last_loop_time))
        self.last_loop_time = now

        # [ZAV 변경] 자동 착륙과 외부 trajectory 발행이 충돌하지 않게 막습니다.
        # NAV_LAND owns trajectory_setpoint during landing. External setpoints
        # otherwise overwrite PX4's descent command and prevent ground detection.
        if not self.setpoints_enabled:
            if not self.lio_safety_latched:
                self.publish_offboard_mode()
            return

        # 최신 키보드 입력 폴링
        self.multi_key.poll_latest()

        # ----------------------------------------------------------------------
        # [핵심 2] Offboard 모드: 상위 플래너 외부 명령 중계 (위치 or 속도)
        # ----------------------------------------------------------------------
        if self.flight_mode == "OFFBOARD_TRAJ":
            self.publish_offboard_mode()
            sp = TrajectorySetpoint()
            sp.timestamp = 0

            # 1. 위치 목표(/zav/goal_pose)가 수신된 경우: 3차원 위치 자율 추종
            if self.ext_goal_pose is not None:
                gx, gy, gz, gyaw = self.ext_goal_pose
                # ROS(North, East, Up) -> PX4 NED(North, East, Down)
                sp.position = [float(gx), float(gy), float(-gz)]
                sp.velocity = [float('nan'), float('nan'), float('nan')]
                sp.acceleration = [float('nan'), float('nan'), float('nan')]
                sp.yaw = float(gyaw) if not math.isnan(gyaw) else float('nan')
                sp.yawspeed = float('nan')
                self.setpoint_pub.publish(sp)
                return

            # 2. 속도 목표(/zav/cmd_vel)가 수신된 경우: 속도 벡터 추종
            elif self.ext_cmd_vel is not None and (now - self.last_ext_vel_time < 0.5):
                vx = self.ext_cmd_vel.linear.x
                vy = self.ext_cmd_vel.linear.y
                vz = self.ext_cmd_vel.linear.z
                yaw_rate = self.ext_cmd_vel.angular.z

                # 좌표계 변환: 상위 플래너 Body(FLU) -> PX4 Inertial(NED)
                psi = self.current_yaw
                vn = vx * math.cos(psi) - vy * math.sin(psi)
                ve = vx * math.sin(psi) + vy * math.cos(psi)
                vd = -vz

                sp.position = [float('nan'), float('nan'), float('nan')]
                sp.velocity = [float(vn), float(ve), float(vd)]
                sp.acceleration = [float('nan'), float('nan'), float('nan')]
                sp.yaw = float('nan')
                sp.yawspeed = float(yaw_rate)
                self.setpoint_pub.publish(sp)
                return

            # 3. 외부 명령 대기 중: 전환 시점 위치에 안전하게 정지 호버링
            else:
                sp.position = [float(self.hold_x), float(self.hold_y), float(self.hold_z)]
                sp.velocity = [0.0, 0.0, 0.0]
                sp.acceleration = [float('nan'), float('nan'), float('nan')]
                sp.yaw = float(self.hold_yaw)
                sp.yawspeed = 0.0
                self.setpoint_pub.publish(sp)
                return

        # ----------------------------------------------------------------------
        # [핵심 1] Manual Teleop 모드 및 자동 이륙 처리
        # ----------------------------------------------------------------------
        self.publish_offboard_mode()
        sp = TrajectorySetpoint()
        sp.timestamp = 0

        # 자동 2.0m 수직 이륙 시퀀스 (T 키)
        if self.is_taking_off:
            # [ZAV 변경] 8초 경과만으로 완료 처리하지 않고 실제 목표 고도를 확인합니다.
            if self.current_alt >= 1.95:
                self.target_yaw = wrap_pi(self.current_yaw)
                print(f"\r\n✨ 2.0m 이륙 완료! (고도: {self.current_alt:.2f}m, 헤딩: {math.degrees(self.target_yaw):.1f}°) -> 수동 조종 가능!\r\n")
                self.is_taking_off = False
                self.manual_hold = (self.takeoff_x, self.takeoff_y, -2.0)
                self.manual_z_hold = -2.0
                self.is_flying = True
                self.des_vx_body = self.cmd_vx_body = 0.0
                self.des_vy_body = self.cmd_vy_body = 0.0
                self.des_vz_ned = self.cmd_vz_ned = 0.0
                self.des_yaw_rate = self.cmd_yaw_rate = 0.0
            else:
                sp.position = [float(self.takeoff_x), float(self.takeoff_y), -2.0]
                sp.velocity = [0.0, 0.0, 0.0]
                sp.yaw = float(self.target_yaw) if self.yaw_initialized else float('nan')
                sp.yawspeed = 0.0
                self.setpoint_pub.publish(sp)
                return

        # 다중 키 입력 적용
        if self.multi_key.active:
            self._update_multi_key_targets()
        else:
            if now > self.vx_release_time: self.des_vx_body = 0.0
            if now > self.vy_release_time: self.des_vy_body = 0.0
            if now > self.vz_release_time: self.des_vz_ned = 0.0
            if now > self.yaw_release_time: self.des_yaw_rate = 0.0

        # Match acceleration feedforward to the manual velocity ramp.
        previous_vx, previous_vy = self.cmd_vx_body, self.cmd_vy_body
        # 스틱 선형 가감속 (Linear Ramp)
        self.cmd_vx_body = ramp_value(self.cmd_vx_body, self.des_vx_body, self.accel_xy, self.decel_xy, dt)
        self.cmd_vy_body = ramp_value(self.cmd_vy_body, self.des_vy_body, self.accel_xy, self.decel_xy, dt)
        self.cmd_vz_ned = ramp_value(self.cmd_vz_ned, self.des_vz_ned, self.accel_z, self.decel_z, dt)
        self.cmd_yaw_rate = ramp_value(self.cmd_yaw_rate, self.des_yaw_rate, self.accel_yaw, self.decel_yaw, dt)

        # 대각선 속도 원형 정규화
        vel_xy_norm = math.hypot(self.cmd_vx_body, self.cmd_vy_body)
        if vel_xy_norm > self.max_speed_xy:
            scale = self.max_speed_xy / vel_xy_norm
            vx_eff = self.cmd_vx_body * scale
            vy_eff = self.cmd_vy_body * scale
        else:
            vx_eff = self.cmd_vx_body
            vy_eff = self.cmd_vy_body

        # 실시간 상태 출력 (조작 중에만 출력)
        is_moving_now = (vel_xy_norm > 0.02) or (abs(self.cmd_vz_ned) > 0.02) or (abs(self.cmd_yaw_rate) > 0.02)
        if (is_moving_now or self.was_moving) and (now - self.last_print_time > 0.08):
            self.last_print_time = now
            self.was_moving = is_moving_now
            print(
                f"\r🎮 [스틱] 수평:{vel_xy_norm:4.1f}m/s (전후:{vx_eff:+5.1f}, 좌우:{vy_eff:+5.1f}) | "
                f"수직:{-self.cmd_vz_ned:+4.1f}m/s | "
                f"Yaw:{math.degrees(self.cmd_yaw_rate):+4.1f}°/s (헤딩:{math.degrees(self.current_yaw):+5.1f}°)   ",
                end='', flush=True
            )

        # [부드러운 Yaw 회전 적분] 0.08 스텝으로 회전 충격 완화
        if abs(self.cmd_yaw_rate) > 0.01:
            self.was_turning_yaw = True
            self.target_yaw = wrap_pi(self.current_yaw + self.cmd_yaw_rate * 0.08)
        else:
            if self.was_turning_yaw:
                self.target_yaw = wrap_pi(self.current_yaw)
                self.was_turning_yaw = False

        # ----------------------------------------------------------------------
        # [핵심 3] PX4 <-> ROS 2 좌표계 변환 (Rule 2 상세 주석)
        # ----------------------------------------------------------------------
        # 기체 로컬 속도 (Body Frame):
        #   - vx_eff: 전진(+) / 후진(-)
        #   - vy_eff: 오른쪽(+) / 왼쪽(-) [보정된 방향]
        #
        # 회전 중일 때는 현재 실시간 헤딩(current_yaw)을 사용하여
        # 기수가 돌아가는 방향으로 전진 속도 벡터가 실시간 선회(Heading Follow)합니다.
        #
        # Inertial NED (North-East-Down) 변환 수식:
        #   vn = vx_eff * cos(psi) - vy_eff * sin(psi)
        #   ve = vx_eff * sin(psi) + vy_eff * cos(psi)
        #   vd = cmd_vz_ned
        # ----------------------------------------------------------------------
        psi = self.current_yaw if self.was_turning_yaw else (self.target_yaw if self.yaw_initialized else self.current_yaw)
        vn = vx_eff * math.cos(psi) - vy_eff * math.sin(psi)
        ve = vx_eff * math.sin(psi) + vy_eff * math.cos(psi)
        vd = self.cmd_vz_ned
        ax_body = (self.cmd_vx_body - previous_vx) / dt
        ay_body = (self.cmd_vy_body - previous_vy) / dt
        acceleration_norm = math.hypot(ax_body, ay_body)
        if acceleration_norm > 4.0:
            ax_body *= 4.0 / acceleration_norm
            ay_body *= 4.0 / acceleration_norm
        an = ax_body * math.cos(psi) - ay_body * math.sin(psi)
        ae = ax_body * math.sin(psi) + ay_body * math.cos(psi)

        # [ZAV 변경] 키를 놓으면 속도 0만 유지하지 않고 정지 위치를 고정합니다.
        # With released sticks, hold a position rather than allowing velocity-mode drift.
        if self.is_flying and not is_moving_now:
            if self.manual_hold is None:
                # Brake with zero velocity first. Capturing a moving position
                # makes the position loop pull the aircraft back after braking.
                if self.current_speed < 0.08:
                    if self.brake_settled_since is None:
                        self.brake_settled_since = time.monotonic()
                    if time.monotonic() - self.brake_settled_since >= 0.3:
                        self.manual_hold = (self.current_x, self.current_y, -self.current_alt)
                else:
                    self.brake_settled_since = None
            sp.position = ([float(v) for v in self.manual_hold] if self.manual_hold is not None
                           else [float('nan')] * 3)
        else:
            self.manual_hold = None
            self.brake_settled_since = None
            sp.position = [float('nan'), float('nan'), float('nan')]
        # Preserve altitude while moving horizontally, including brake phase.
        if self.is_flying and abs(self.cmd_vz_ned) < 0.02 and abs(self.des_vz_ned) < 0.02:
            if self.manual_z_hold is None:
                self.manual_z_hold = -self.current_alt
            sp.position[2] = float(self.manual_z_hold)
        else:
            self.manual_z_hold = None
        sp.velocity = [float(vn), float(ve), float(vd)]
        sp.acceleration = [float(an), float(ae), float('nan')]
        sp.yaw = float('nan') if self.was_turning_yaw else (float(self.target_yaw) if self.yaw_initialized else float('nan'))
        sp.yawspeed = float(self.cmd_yaw_rate)
        self.setpoint_pub.publish(sp)

    # --------------------------------------------------------------------------
    # 사용자 안내 및 키보드 이벤트 처리
    # --------------------------------------------------------------------------
    def _print_guide(self):
        print(f"""
======================================================================
   🚁 [ZAV Flight Manager - Clean Bridge & Safety Gateway] 🚁
======================================================================
  [비행 명령 (터미널 창에서 입력)]
    Spacebar : 시동 토글 (Arm / Disarm) 및 긴급 제자리 브레이크
    T        : 자동 수직 이륙 (목표 고도 2.0m 도달 후 호버링)
    1        : Manual Teleop 모드 (수동 조종 활성화)
    2        : Offboard 모드 (상위 플래너 /zav/goal_pose 자율비행 / 수동 키 잠금 ⛔)
    L        : 자동 착륙 (Land)
    E        : 비상 모터 강제 전원 차단 (Emergency Kill)
    Q        : 안전 종료

  [수동 조종키 (드론 기수 로컬 프레임 기준)]
    W / S    : 고도 상승(최대 3.0 m/s) / 하강(최대 1.5 m/s)
    A / D    : 부드러운 좌회전 / 우회전 (최대 57°/s 부드러운 선회)
    ↑ / ↓    : 드론 기수 앞(전진) / 뒤(후진), 최대 {self.max_speed_xy:g} m/s
    ← / →    : 드론 기수 왼쪽 / 오른쪽 (방향 보정 완료)
    ★ 회전 선회 : 전진(↑)하면서 좌/우 회전(A/D) 시 기수 방향으로 부드럽게 선회!
    ★ 대각선   : [↑ + ←], [↑ + →] 등 다중 키 동시 입력 지원
======================================================================
입력 대기 중...
""")

    def _extend_hold_time(self, current_release_time: float, now: float) -> float:
        return (now + 0.58) if now > current_release_time else (now + 0.18)

    def update_teleop(self, key: str) -> bool:
        now = time.time()
        # Preserve explicit emergency stop/exit, but never resume offboard after a latch.
        if self.lio_safety_latched and key not in ('e', 'E', 'q', 'Q', '\x03'):
            return True

        if key in ('t', 'T'):
            if (self.lio_safety_ready is False or
                    (self.lio_safety_rx is not None and time.monotonic() - self.lio_safety_rx > 1.0)):
                print('\r\n이륙 대기: LIO safety가 READY가 아닙니다. /zav/lio_safety/status 확인.\r\n')
                return True
            if not self.position_ready or time.monotonic() - self.last_position_time > 1.0:
                if self.last_position_time == 0.0:
                    reason = "PX4 위치 메시지 미수신 — 시뮬레이터의 DDS 게이트웨이를 확인하세요."
                elif time.monotonic() - self.last_position_time > 1.0:
                    reason = "PX4 위치 수신 중단 — 시뮬레이터 실행/일시정지 상태를 확인하세요."
                else:
                    reason = "PX4 위치 추정이 아직 유효하지 않습니다. Ready for takeoff를 기다리세요."
                print(f"\r\n이륙 대기: {reason}\r\n")
                return True
            print("\r\n🚀 [이륙] 시동(Arm) & 2.0m 자동 수직 이륙 시작...\r\n")
            self.setpoints_enabled = True
            self.flight_mode = "MANUAL"
            self.ext_goal_pose = None
            self.manual_hold = None
            self.manual_z_hold = None
            self.takeoff_x = self.current_x
            self.takeoff_y = self.current_y
            self.target_yaw = wrap_pi(self.current_yaw)
            self.yaw_initialized = True
            self.des_vx_body = self.cmd_vx_body = 0.0
            self.des_vy_body = self.cmd_vy_body = 0.0
            self.des_vz_ned = self.cmd_vz_ned = 0.0
            self.des_yaw_rate = self.cmd_yaw_rate = 0.0
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            time.sleep(0.05)
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.is_taking_off = True
            self.is_flying = True

        elif key in ('1',):
            self.manual_hold = None
            self.manual_z_hold = -self.current_alt
            self.brake_settled_since = None
            self.flight_mode = "MANUAL"
            self.ext_goal_pose = None
            print("\r\n🎮 [모드 전환] -> 1번 Manual Teleop 모드 (수동 조종 활성화)\r\n")

        elif key in ('2',):
            self.flight_mode = "OFFBOARD_TRAJ"
            self.des_vx_body = self.cmd_vx_body = 0.0
            self.des_vy_body = self.cmd_vy_body = 0.0
            self.des_vz_ned = self.cmd_vz_ned = 0.0
            self.des_yaw_rate = self.cmd_yaw_rate = 0.0
            # 전환 시점의 현재 위치를 안전 호버링 위치로 고정
            self.hold_x = self.current_x
            self.hold_y = self.current_y
            self.hold_z = -self.current_alt if self.current_alt > 0.3 else -2.0
            self.hold_yaw = self.current_yaw
            print(f"\r\n🤖 [모드 전환] -> 2번 Offboard 자율비행 모드 (/zav/goal_pose 수신 대기 / 수동 키 잠금 ⛔)")
            print(f"   현재 위치 유지 호버링: X={self.hold_x:.2f}m, Y={self.hold_y:.2f}m, Z={-self.hold_z:.2f}m\r\n")

        elif key in ('l', 'L'):
            print("\r\n🛬 [착륙] 자동 착륙(NAV_LAND) 명령 송신...\r\n")
            self.setpoints_enabled = False
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.des_vx_body = self.cmd_vx_body = 0.0
            self.des_vy_body = self.cmd_vy_body = 0.0
            self.des_vz_ned = self.cmd_vz_ned = 0.0
            self.des_yaw_rate = self.cmd_yaw_rate = 0.0
            self.is_flying = False
            self.is_taking_off = False

        elif key == ' ':
            self.des_vx_body = self.cmd_vx_body = 0.0
            self.des_vy_body = self.cmd_vy_body = 0.0
            self.des_vz_ned = self.cmd_vz_ned = 0.0
            self.des_yaw_rate = self.cmd_yaw_rate = 0.0
            self.vx_release_time = self.vy_release_time = self.vz_release_time = self.yaw_release_time = 0.0
            self.target_yaw = wrap_pi(self.current_yaw)
            self.ext_goal_pose = None
            if not self.is_flying and not self.is_taking_off:
                if not self.is_armed:
                    print("\r\n⚡ [시동] Arming 명령 송신...\r\n")
                    self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                else:
                    print("\r\n🛑 [시동 끄기] Disarming 명령 송신...\r\n")
                    self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            else:
                print(f"\r\n🛑 [브레이크] 즉시 제자리 호버링 고정! (고도: {self.current_alt:.2f}m, 헤딩: {math.degrees(self.target_yaw):.1f}°)\r\n")

        elif key in ('e', 'E'):
            self.setpoints_enabled = False
            print("\r\n🚨 [비상 정지] 모터 강제 전원 차단 (Kill Switch)...\r\n")
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
            self.is_flying = False
            self.is_taking_off = False

        elif key in ('q', 'Q', '\x03'):
            print("\r\n🛑 [종료] 노드를 안전하게 종료합니다...\r\n")
            if self.is_flying:
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            time.sleep(0.3)
            return False

        # 리눅스 터미널 폴백 모드 (Win32 파이프 미작동 시)
        if not self.multi_key.active and self.flight_mode == "MANUAL":
            if key in ('w', 'W'):
                self.vz_release_time = self._extend_hold_time(self.vz_release_time, now)
                self.des_vz_ned = -self.max_speed_z_up
            elif key in ('s', 'S'):
                self.vz_release_time = self._extend_hold_time(self.vz_release_time, now)
                self.des_vz_ned = self.max_speed_z_dn
            elif key in ('a', 'A'):
                self.yaw_release_time = self._extend_hold_time(self.yaw_release_time, now)
                self.des_yaw_rate = -self.max_yaw_rate
            elif key in ('d', 'D'):
                self.yaw_release_time = self._extend_hold_time(self.yaw_release_time, now)
                self.des_yaw_rate = self.max_yaw_rate
            elif key in ('\x1b[A', '\x1bOA'):
                self.vx_release_time = self._extend_hold_time(self.vx_release_time, now)
                self.des_vx_body = self.max_speed_xy
            elif key in ('\x1b[B', '\x1bOB'):
                self.vx_release_time = self._extend_hold_time(self.vx_release_time, now)
                self.des_vx_body = -self.max_speed_xy
            elif key in ('\x1b[D', '\x1bOD'):  # 왼쪽 화살표
                self.vy_release_time = self._extend_hold_time(self.vy_release_time, now)
                self.des_vy_body = -self.max_speed_xy
            elif key in ('\x1b[C', '\x1bOC'):  # 오른쪽 화살표
                self.vy_release_time = self._extend_hold_time(self.vy_release_time, now)
                self.des_vy_body = self.max_speed_xy

        return True


def main(args=None):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    # TCP discovery has no multicast: planners must know this participant's port.
    # [ZAV 변경] WSL TCP 선택 시에만 고정 포트 설정. Ubuntu/hardware DDS 설정은 덮어쓰지 않습니다.
    if os.environ.get("ZAV_ROS_TRANSPORT") == "cyclone_tcp":
        dds_config = Path(__file__).with_name("cyclonedds_flight_manager.xml")
        os.environ["CYCLONEDDS_URI"] = dds_config.as_uri()
    rclpy.init(args=args)
    node = ZAVFlightManager()

    try:
        tty.setraw(fd, termios.TCSANOW)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            key = read_key_nonblocking(fd)
            if key != '':
                if not node.update_teleop(key):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        node.multi_key.stop()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\n✅ ZAV Flight Manager가 안전하게 종료되었습니다.")


if __name__ == '__main__':
    main()
