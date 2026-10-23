#!/usr/bin/env python3
"""
==============================================================================
ZAV Quadrotor Multi-Key Teleoperation Node (Zero-Lag & Full Spec)
==============================================================================
- 기체: 26 SfUAV ZAV 커스텀 쿼드콥터 (1.99 kg, T-Motor F90 6S, 추력비 4.2:1)
- 핵심 알고리즘 및 물리 원리 (Rule 2 상세 주석):
  1. 무지연(Zero-Lag) 하드웨어 다중 키 동시 입력:
     Windows user32.dll GetAsyncKeyState를 통해 현재 눌려 있는 모든 키의 비트마스크를
     스트리밍하되, 파이썬에서 논블로킹 os.read(fd, 4096)으로 파이프 버퍼를 매번
     완전히 비우고 가장 최신 값(lines[-1])만 취합니다.
     이를 통해 파이프 큐 밀림(Buffer Bloat)을 원천 제거하여, 키에서 손을 떼는 즉시
     0.01초 만에 목표 속도가 0.0으로 복귀하고 대각선 동시 입력이 완벽히 작동합니다.

  2. 대각선 속도 벡터 원형 정규화 (Circular Stick Gate Normalization):
     전후 속도(vx)와 좌우 속도(vy)가 동시에 인가될 때 대각선 합속도의 크기가
     최대 스펙(v_max = 12.0 m/s)을 초과하지 않도록 2D 유클리드 노름으로 정규화합니다.

  3. ZAV 실기체 & PX4 최대 속도 스펙 반영 및 선형 스틱 가감속(Linear Ramp):
     - 수평 최대 속도 (MPC_XY_VEL_MAX): 12.0 m/s (시속 43.2 km/h)
     - 상승 최대 속도 (MPC_Z_VEL_MAX_UP): 3.0 m/s
     - 하강 최대 속도 (MPC_Z_VEL_MAX_DN): 1.5 m/s
     - 회전 최대 각속도: 2.5 rad/s (약 143.2 deg/s)
==============================================================================
"""

import os
import sys
import base64
import termios
import tty
import select
import time
import math
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# PX4 uORB 메시지 인터페이스
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition


# 비트마스크 상수 정의
KEY_UP    = 1 << 0  # 1   (전진 ↑)
KEY_DOWN  = 1 << 1  # 2   (후진 ↓)
KEY_LEFT  = 1 << 2  # 4   (좌측 ←)
KEY_RIGHT = 1 << 3  # 8   (우측 →)
KEY_W     = 1 << 4  # 16  (상승 W)
KEY_S     = 1 << 5  # 32  (하강 S)
KEY_A     = 1 << 6  # 64  (좌회전 A)
KEY_D     = 1 << 7  # 128 (우회전 D)
KEY_SPACE = 1 << 8  # 256 (긴급 브레이크 Space)


def wrap_pi(angle: float) -> float:
    """각도를 [-pi, +pi] 범위로 정규화"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def ramp_value(current: float, target: float, accel_rate: float, decel_rate: float, dt: float) -> float:
    """
    [스틱 선형 가감속 함수 (Linear Ramp / Slew-Rate Limiter)]
    - 스틱을 밀 때(가속)는 accel_rate 기울기로 0에서 Max까지 선형 증가하고,
    - 스틱을 놓거나 반대 방향으로 꺾을 때(감속)는 decel_rate 기울기로 신속하고 부드럽게 0으로 복귀합니다.
    """
    if abs(target) < 1e-5 or (current * target < 0.0):
        rate = decel_rate
    else:
        rate = accel_rate

    step = rate * dt
    if current < target:
        return min(target, current + step)
    elif current > target:
        return max(target, current - step)
    return target


class Win32MultiKeyReader:
    """
    WSL2 환경에서 Windows user32.dll GetAsyncKeyState를 호출하여 다중 키 동시 눌림을 감지하되,
    파이프 버퍼를 논블로킹으로 한 번에 모두 비우고 최신 상태(lines[-1])만 읽어
    키를 뗐을 때 지연 없이 즉시 0으로 복귀하도록 보장하는 클래스.
    """
    def __init__(self):
        self.key_mask = 0
        self.active = False
        self._proc = None
        self._pipe_fd = None
        self._recv_buf = b""

        ps_exe = "/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"
        if os.path.exists(ps_exe):
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
                # stdin=subprocess.DEVNULL로 리눅스 터미널 입력(T, L, Q 등)을 절대 가로채지 않도록 격리
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
        """파이프에 쌓인 모든 데이터를 한 번에 비우고 가장 마지막(최신) 비트마스크만 추출"""
        if self._pipe_fd is None:
            return
        try:
            while True:
                data = os.read(self._pipe_fd, 4096)
                if not data:
                    break
                self._recv_buf += data
        except BlockingIOError:
            pass
        except Exception:
            return

        if b"\n" in self._recv_buf:
            lines = self._recv_buf.split(b"\n")
            self._recv_buf = lines[-1]
            for raw_line in reversed(lines[:-1]):
                s = raw_line.strip()
                if s.isdigit():
                    self.key_mask = int(s)
                    self.active = True
                    break

    def stop(self):
        if self._proc:
            try:
                self._proc.kill()
            except Exception:
                pass


class ZAVTeleopNode(Node):
    def __init__(self):
        super().__init__('zav_teleop_keyboard')

        # ----------------------------------------------------------------------
        # 1. ZAV 실기체 & PX4 최대 속도 스펙 및 선형 가감속 파라미터
        # ----------------------------------------------------------------------
        self.is_armed = False
        self.nav_state = 0
        self.is_flying = False
        self.is_taking_off = False
        self.takeoff_start_time = 0.0

        # [1] 수평 비행 스펙 (PX4 MPC_XY_VEL_MAX = 12.0 m/s)
        self.max_speed_xy = 12.0     # 최대 수평 속도: 12.0 m/s (시속 43.2 km/h)
        self.accel_xy = 5.0          # 선형 가속율: 5.0 m/s^2 (누르고 있으면 0 -> 12 m/s 선형 증가)
        self.decel_xy = 14.0         # 선형 감속율: 14.0 m/s^2 (손 떼면 빠르게 0.0으로 브레이크 복귀!)

        # [2] 수직 비행 스펙 (PX4 MPC_Z_VEL_MAX_UP = 3.0 m/s, DN = 1.5 m/s)
        self.max_speed_z_up = 3.0    # 최대 상승 속도: 3.0 m/s
        self.max_speed_z_dn = 1.5    # 최대 하강 속도: 1.5 m/s
        self.accel_z = 2.5           # 수직 가속율: 2.5 m/s^2
        self.decel_z = 6.0           # 수직 감속율: 6.0 m/s^2

        # [3] Yaw 회전 스펙
        self.max_yaw_rate = 2.5      # 최대 회전 각속도: 2.5 rad/s (약 143.2 deg/s)
        self.accel_yaw = 2.5         # 회전 선형 가속율: 2.5 rad/s^2
        self.decel_yaw = 8.0         # 회전 선형 감속율: 8.0 rad/s^2 (손 떼면 즉시 0.0 정지!)

        # 센서 피드백
        self.current_alt = 0.0
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.takeoff_x = 0.0
        self.takeoff_y = 0.0
        self.yaw_initialized = False

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

        # 무지연 다중 키 동시 입력 리더 시작
        self.multi_key = Win32MultiKeyReader()
        self.last_print_time = 0.0
        self.was_moving = False

        # 순수 리눅스 폴백용 타이머
        self.vx_release_time = 0.0
        self.vy_release_time = 0.0
        self.vz_release_time = 0.0
        self.yaw_release_time = 0.0

        # ----------------------------------------------------------------------
        # 2. QoS 프로파일 (PX4 uORB 규격)
        # ----------------------------------------------------------------------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ----------------------------------------------------------------------
        # 3. 퍼블리셔 및 서브스크라이버
        # ----------------------------------------------------------------------
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self._status_cb, qos_profile
        )
        self.status_v4_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self._status_cb, qos_profile
        )
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._pos_cb, qos_profile
        )
        self.local_pos_v1_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._pos_cb, qos_profile
        )

        # 50Hz (20ms) 고정밀 제어 루프
        self.dt = 0.02
        self.last_loop_time = time.time()
        self.timer = self.create_timer(self.dt, self._control_loop)

        self._print_guide()

    def _status_cb(self, msg: VehicleStatus):
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state

    def _pos_cb(self, msg: VehicleLocalPosition):
        self.current_alt = -msg.z
        self.current_yaw = msg.heading
        self.current_x = msg.x
        self.current_y = msg.y
        if not self.yaw_initialized:
            self.target_yaw = wrap_pi(msg.heading)
            self.yaw_initialized = True
        if self.current_alt > 0.5 and not self.is_taking_off and not self.is_flying:
            self.is_flying = True
            self.target_yaw = wrap_pi(msg.heading)

    def publish_offboard_mode(self):
        """PX4에게 속도 및 위치 제어 모드임을 지속 송신"""
        mode = OffboardControlMode()
        mode.position = True
        mode.velocity = True
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.timestamp = 0
        self.offboard_mode_pub.publish(mode)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """PX4 시스템 명령 전송"""
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = param1
        cmd.param2 = param2
        cmd.param7 = param7
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = 0
        self.cmd_pub.publish(cmd)

    def _update_multi_key_targets(self):
        """
        [무지연 다중 키 동시 입력 처리]
        현재 눌려 있는 키는 목표 속도를 Max로 설정하고,
        손을 뗀 키는 즉시 목표 속도(des_*)를 0.0으로 복귀시킵니다.
        """
        mask = self.multi_key.key_mask

        # Spacebar: 긴급 브레이크 (모든 속도 즉시 0)
        if mask & KEY_SPACE:
            self.des_vx_body = self.cmd_vx_body = 0.0
            self.des_vy_body = self.cmd_vy_body = 0.0
            self.des_vz_ned = self.cmd_vz_ned = 0.0
            self.des_yaw_rate = self.cmd_yaw_rate = 0.0
            self.target_yaw = wrap_pi(self.current_yaw)
            return

        # 1) 전후 축 (Up / Down) - 안 눌려 있으면 즉시 0.0!
        if (mask & KEY_UP) and not (mask & KEY_DOWN):
            self.des_vx_body = self.max_speed_xy
        elif (mask & KEY_DOWN) and not (mask & KEY_UP):
            self.des_vx_body = -self.max_speed_xy
        else:
            self.des_vx_body = 0.0

        # 2) 좌우 축 (Left / Right) - 안 눌려 있으면 즉시 0.0!
        if (mask & KEY_LEFT) and not (mask & KEY_RIGHT):
            self.des_vy_body = self.max_speed_xy
        elif (mask & KEY_RIGHT) and not (mask & KEY_LEFT):
            self.des_vy_body = -self.max_speed_xy
        else:
            self.des_vy_body = 0.0

        # 3) 수직 축 (W / S) - 안 눌려 있으면 즉시 0.0!
        if (mask & KEY_W) and not (mask & KEY_S):
            self.des_vz_ned = -self.max_speed_z_up
        elif (mask & KEY_S) and not (mask & KEY_W):
            self.des_vz_ned = self.max_speed_z_dn
        else:
            self.des_vz_ned = 0.0

        # 4) Yaw 회전 축 (A / D) - 안 눌려 있으면 즉시 0.0!
        if (mask & KEY_A) and not (mask & KEY_D):
            self.des_yaw_rate = -self.max_yaw_rate
        elif (mask & KEY_D) and not (mask & KEY_A):
            self.des_yaw_rate = self.max_yaw_rate
        else:
            self.des_yaw_rate = 0.0

    def _control_loop(self):
        now = time.time()
        dt = min(0.1, max(0.005, now - self.last_loop_time))
        self.last_loop_time = now

        # 파이프 버퍼를 모두 비우고 현재 순간의 최신 키 상태 업데이트
        self.multi_key.poll_latest()

        self.publish_offboard_mode()

        sp = TrajectorySetpoint()
        sp.timestamp = 0

        # 1. 이륙 시퀀스: 수평 위치 사수하며 수직 상승
        if self.is_taking_off:
            if (self.current_alt >= 1.8) or (now - self.takeoff_start_time > 8.0):
                self.target_yaw = wrap_pi(self.current_yaw)
                print(f"\r\n✨ 2.0m 이륙 완료! (고도: {self.current_alt:.2f}m, 헤딩: {math.degrees(self.target_yaw):.1f}°) -> 다중키 조종 가능!\r")
                self.is_taking_off = False
                self.is_flying = True
                self.des_vx_body = self.cmd_vx_body = 0.0
                self.des_vy_body = self.cmd_vy_body = 0.0
                self.des_vz_ned = self.cmd_vz_ned = 0.0
                self.des_yaw_rate = self.cmd_yaw_rate = 0.0
            else:
                sp.position = [float(self.takeoff_x), float(self.takeoff_y), -2.0]
                sp.velocity = [0.0, 0.0, -1.5]
                sp.yaw = float(self.target_yaw) if self.yaw_initialized else float('nan')
                sp.yawspeed = 0.0
                self.setpoint_pub.publish(sp)
                return

        # 2. 다중 키 동시 입력 상태 업데이트 (활성화 시) 또는 폴백 타이머 체크
        if self.multi_key.active:
            self._update_multi_key_targets()
        else:
            if now > self.vx_release_time:
                self.des_vx_body = 0.0
            if now > self.vy_release_time:
                self.des_vy_body = 0.0
            if now > self.vz_release_time:
                self.des_vz_ned = 0.0
            if now > self.yaw_release_time:
                self.des_yaw_rate = 0.0

        # 3. 스틱 선형 가감속(Linear Ramp) 계산: 누르면 0->Max 가속, 떼면 Max->0 즉시 감속!
        self.cmd_vx_body = ramp_value(self.cmd_vx_body, self.des_vx_body, self.accel_xy, self.decel_xy, dt)
        self.cmd_vy_body = ramp_value(self.cmd_vy_body, self.des_vy_body, self.accel_xy, self.decel_xy, dt)
        self.cmd_vz_ned = ramp_value(self.cmd_vz_ned, self.des_vz_ned, self.accel_z, self.decel_z, dt)
        self.cmd_yaw_rate = ramp_value(self.cmd_yaw_rate, self.des_yaw_rate, self.accel_yaw, self.decel_yaw, dt)

        # 4. 대각선 이동 시 최대 수평 속도(12.0 m/s) 원형 정규화 (Circular Clamping)
        vel_xy_norm = math.hypot(self.cmd_vx_body, self.cmd_vy_body)
        if vel_xy_norm > self.max_speed_xy:
            scale = self.max_speed_xy / vel_xy_norm
            vx_eff = self.cmd_vx_body * scale
            vy_eff = self.cmd_vy_body * scale
        else:
            vx_eff = self.cmd_vx_body
            vy_eff = self.cmd_vy_body

        # 실시간 속도 상태 출력 (가감속 중이거나 정지 순간까지 실시간 표시)
        is_moving_now = (vel_xy_norm > 0.02) or (abs(self.cmd_vz_ned) > 0.02) or (abs(self.cmd_yaw_rate) > 0.02)
        if (is_moving_now or self.was_moving) and (now - self.last_print_time > 0.08):
            self.last_print_time = now
            self.was_moving = is_moving_now
            print(
                f"\r🎮 [스틱] 수평:{vel_xy_norm:4.1f}m/s (전후:{vx_eff:+5.1f}, 좌우:{vy_eff:+5.1f}) | "
                f"수직:{-self.cmd_vz_ned:+4.1f}m/s | "
                f"Yaw:{math.degrees(self.cmd_yaw_rate):+5.1f}°/s (헤딩:{math.degrees(self.current_yaw):+6.1f}°)   ",
                end='', flush=True
            )

        # 5. Yaw 회전 적분 및 헤딩 고정 처리
        if abs(self.cmd_yaw_rate) > 0.01:
            self.was_turning_yaw = True
            self.target_yaw = wrap_pi(self.current_yaw + self.cmd_yaw_rate * 0.15)
        else:
            if self.was_turning_yaw:
                self.target_yaw = wrap_pi(self.current_yaw)
                self.was_turning_yaw = False

        # 6. 현재 기체 헤딩 기준 Body(FLU) -> Inertial(NED) 속도 변환
        psi = self.current_yaw if self.was_turning_yaw else (self.target_yaw if self.yaw_initialized else self.current_yaw)
        vn = vx_eff * math.cos(psi) + vy_eff * math.sin(psi)
        ve = vx_eff * math.sin(psi) - vy_eff * math.cos(psi)
        vd = self.cmd_vz_ned

        # 7. TrajectorySetpoint 송신
        sp.position = [float('nan'), float('nan'), float('nan')]
        sp.velocity = [float(vn), float(ve), float(vd)]
        sp.acceleration = [float('nan'), float('nan'), float('nan')]
        sp.yaw = float('nan') if self.was_turning_yaw else (float(self.target_yaw) if self.yaw_initialized else float('nan'))
        sp.yawspeed = float(self.cmd_yaw_rate)
        self.setpoint_pub.publish(sp)

    def _print_guide(self):
        print("""
======================================================================
   🚁 [ZAV 실기체 풀스펙 & 무지연 다중키 동시입력 조종 콘솔] 🚁
======================================================================
  [비행 명령 (터미널 창에서 입력)]
    T        : 자동 수직 이륙 (2.0m 도달 후 호버링)
    L        : 자동 착륙 (Land)
    Spacebar : 긴급 정지 브레이크 (즉시 모든 속도 0)
    Q        : 착륙 후 종료

  [왼쪽 스틱 - 고도 및 Yaw 회전 (누르면 선형 가속 / 떼면 즉시 0 복귀)]
    W / S    : 상승(최대 3.0 m/s) / 하강(최대 1.5 m/s)
    A / D    : 좌회전 / 우회전 (0 -> 최대 143°/s 선형 가속)

  [오른쪽 스틱 - 수평 및 대각선 비행 (누르면 선형 가속 / 떼면 즉시 0 복귀)]
    ↑ / ↓    : 전진 / 후진 (0 -> 최대 12.0 m/s [43km/h] 선형 가속)
    ← / →    : 좌측 / 우측 (0 -> 최대 12.0 m/s 선형 가속)
    ★ 대각선 : [↑ + ←], [↑ + →] 등 여러 키 동시 입력 완벽 지원!
======================================================================
입력 대기 중... (T 키를 눌러 이륙하세요)
""")

    def _extend_hold_time(self, current_release_time: float, now: float) -> float:
        if now > current_release_time:
            return now + 0.58
        else:
            return now + 0.18

    def update_teleop(self, key: str):
        now = time.time()

        if key in ('t', 'T'):
            print("\r\n🚀 [이륙] 시동(Arm) & 수직 이륙 시작 (목표 고도 2.0m)...\r")
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
            self.takeoff_start_time = now

        elif key in ('l', 'L'):
            print("\r\n🛬 [착륙] 자동 착륙(NAV_LAND) 명령 송신...\r")
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
            print(f"\r\n🛑 [브레이크] 즉시 호버링 고정! (고도: {self.current_alt:.2f}m, 헤딩: {math.degrees(self.target_yaw):.1f}°)\r")

        elif key in ('q', 'Q', '\x03'):
            print("\r\n🛑 [종료] 자동 착륙 명령 후 노드를 안전하게 종료합니다...\r")
            if self.is_flying:
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            time.sleep(0.5)
            return False

        # 폴백 모드(Win32 비활성 시)에만 stdin으로 이동 처리
        if not self.multi_key.active:
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
            elif key in ('\x1b[D', '\x1bOD'):
                self.vy_release_time = self._extend_hold_time(self.vy_release_time, now)
                self.des_vy_body = self.max_speed_xy
            elif key in ('\x1b[C', '\x1bOC'):
                self.vy_release_time = self._extend_hold_time(self.vy_release_time, now)
                self.des_vy_body = -self.max_speed_xy

        return True


def read_key_nonblocking(fd: int) -> str:
    rlist, _, _ = select.select([fd], [], [], 0.005)
    if not rlist:
        return ''
    raw_bytes = os.read(fd, 32)
    return raw_bytes.decode('utf-8', errors='ignore')


def main(args=None):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    rclpy.init(args=args)
    node = ZAVTeleopNode()

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
        print("\n✅ 조종 노드가 정상적으로 종료되었습니다.")


if __name__ == '__main__':
    main()
