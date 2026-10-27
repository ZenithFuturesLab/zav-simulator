#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
================================================================================
ZAV Drone Simulation on Lightweight Ground Plane with PX4 SITL & Pegasus
================================================================================
설명:
  대용량 야외 맵(지형/텍스처) 로딩으로 인한 디스크 I/O 및 GPU/CPU 병목을 제거하고,
  Isaac Sim 내장 기본 평면(Default Ground Plane) 환경에서 커스텀 드론 ZAV (1.99 kg)를
  물리 200 Hz·렌더링 20 Hz로 시뮬레이션 및 비행 제어하는 런처 스크립트입니다.

물리 제원:
  - 기체: Fusion 360 기반 커스텀 ZAV 쿼드콥터 (프레임, 가드, 카메라, 배터리 포함)
  - 총 질량: 1.99 kg
  - 무게중심(CoM): Z = -0.0182 m
  - 모터 축 간격: 반경 140.0 mm (Wheelbase 280.0 mm, dx = ±99.0 mm, dy = ±99.0 mm)
  - 통신: PX4 MAVLink Backend (TCP 4560 포트)
================================================================================
"""

import os
import sys
from pathlib import Path
# Resolve project paths from this script, independent of the launch directory.
_SIMULATOR_DIR = Path(__file__).resolve().parent
_WORKSPACE_DIR = _SIMULATOR_DIR.parent
sys.path.insert(0, str(_SIMULATOR_DIR))

# [ZAV 변경] Windows에만 필요한 내부 ROS DLL 경로입니다. Linux 라이브러리는 실행기가 설정합니다.
# Windows python.bat sets ISAAC_PATH but does not add the ROS 2 runtime DLLs.
# This must happen before SimulationApp/bridge startup, including its child check process.
_ros_dll_handles = []
if sys.platform == "win32":
    _isaac_root = os.environ.get("ISAAC_PATH")
    if not _isaac_root:
        raise RuntimeError("Run this script with Isaac Sim python.bat (ISAAC_PATH is missing).")
    _ros_lib = os.path.join(
        _isaac_root, "exts", "isaacsim.ros2.bridge", "humble", "lib"
    )
    if not os.path.isdir(_ros_lib):
        raise FileNotFoundError(f"Isaac Sim internal ROS 2 libraries not found: {_ros_lib}")
    os.environ["ROS_DISTRO"] = "humble"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["PATH"] = _ros_lib + os.pathsep + os.environ.get("PATH", "")
    _ros_dll_handles.append(os.add_dll_directory(_ros_lib))
    print(f"[ROS2] Windows internal Humble runtime: {_ros_lib}", flush=True)

from isaacsim import SimulationApp

# 정상 작동하던 기존 설정으로 복원
simulation_app = SimulationApp({"headless": False, "sync_loads": False})

# 초기화 중 확장 모듈 로딩 충돌 여부 확인
for _ in range(5):
    simulation_app.update()

from isaacsim.core.utils.extensions import enable_extension

print("[INIT] RTX extension enabling...", flush=True)
enable_extension("isaacsim.sensors.rtx")

for _ in range(5):
    simulation_app.update()

print("[INIT] ROS2 extension enabling...", flush=True)
enable_extension("isaacsim.ros2.bridge")

for _ in range(5):
    simulation_app.update()

print("[INIT] ROS2 importing...", flush=True)
import rclpy
print("[INIT] ROS2 OK", flush=True)

import carb.input
import carb.settings
import omni.appwindow
import omni.timeline
import omni.usd
from omni.isaac.core.world import World
import omni.isaac.core.utils.prims as prim_utils

import numpy as np
from scipy.spatial.transform import Rotation

# Pegasus Simulator 핵심 API
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotors import ZAV_v1
from pegasus.simulator.params import ASSET_PATH, SIMULATION_ENVIRONMENTS

# 장착 위치는 body 기준 미터, 방향은 Z/Y/X 순서 degree.
# 측정 거리·채널·스캔 속도는 USD의 OmniLidar 속성을 사용합니다.
# [ZAV 변경] 기본 기체/추력은 유지하고 센서 획득 주기만 정수 물리 스텝에 맞춥니다.
PHYSICS_HZ = 160.0  # Mid360 IMU rate; exactly 10 physics ticks per render frame.
RENDER_HZ = 20.0
# SLAM_TEST_SCENE = os.environ.get("PEGASUS_SLAM_SCENE", "1") != "0"  # 선택적 SLAM 검증 장면.
SLAM_TEST_SCENE = False

# [ZAV 변경] 새 센서는 이 설정을 ZAV_v1에 주입합니다. 기존 PX4 센서 목록은 교체하지 않습니다.
MID360_CONFIG = {
    "usd_path": str(Path(ASSET_PATH) / "Sensors" / "Mid-360s.usda"),
    "sensor_prim_path": "/World/Example_Rotary",
    # ZAV USD MeshInstance_151: visual center, tilted 22 degrees toward body +X.
    "position": [0.01215, 0.0, 0.11503],
    "orientation": [0.0, 22.0, 0.0],  # Z/Y/X degrees
    "frequency": RENDER_HZ,
    "show_render": False,
    "publish_laserscan": False,
    "publish_method": "python",
    "tcp_port": 17441,
    "livox_tcp_port": 17445,
    "livox_scan_lines": 4,
}

# FAST-LIVO's real-sensor calibration is the source of truth. Only the LiDAR's
# absolute body mount is anchored to the visual model; other mounts are derived.
from pegasus.simulator.logic.sensors.fastlivo_calibration import load_calibration
FASTLIVO_CONFIG_DIR = os.environ.get(
    "PEGASUS_FASTLIVO_CONFIG_DIR", str(_WORKSPACE_DIR / "src" / "FAST-LIVO2" / "config")
)
_sensor_calibration = load_calibration(
    FASTLIVO_CONFIG_DIR, MID360_CONFIG["position"], MID360_CONFIG["orientation"]
)
MID360_CONFIG["livox_scan_lines"] = _sensor_calibration["scan_lines"]

FRONT_CAMERA_CONFIG = {
    # Relative pose is determined by FAST-LIVO Rcl/Pcl, not CAD mesh bounds.
    "position": _sensor_calibration["camera_position"],
    "orientation": _sensor_calibration["camera_orientation"],
    "resolution": _sensor_calibration["resolution"],
    "intrinsics": _sensor_calibration["intrinsics"],
    "distortion_coefficients": _sensor_calibration["distortion"],
    "frequency": RENDER_HZ,  # Must divide RENDER_HZ; currently one image per render frame.
    "depth": False,
    "publish_method": "python",
    "tcp_port": 17443,
}

MID360_IMU_CONFIG = {
    "position": _sensor_calibration["imu_position"],
    "orientation": _sensor_calibration["imu_orientation"],
    "update_rate": PHYSICS_HZ,  # One measurement per physics tick, independent of rendering.
    "frame_id": "mid360_imu",
    "topic": "mid360/imu",
    "tcp_port": 17444,
    "gyro_noise_density": 0.0,  # Ideal model; not calibrated Livox noise parameters.
    "accel_noise_density": 0.0,
}

class ZAVPlaneApp:
    def __init__(self):
        # 1. 시뮬레이션 타임라인 인터페이스 획득
        self.timeline = omni.timeline.get_timeline_interface()
        
        # 2. 물리/IMU 200 Hz, 센서 렌더링 20 Hz로 설정합니다.
        self.pg = PegasusInterface()
        self.pg._world_settings = {
            **self.pg._world_settings,
            "physics_dt": 1.0 / PHYSICS_HZ,
            "rendering_dt": 1.0 / RENDER_HZ,
        }
        # Limit wall-clock rendering too, rather than only increasing simulated dt.
        settings = carb.settings.get_settings()
        settings.set("/app/runLoops/main/rateLimitEnabled", True)
        settings.set("/app/runLoops/main/rateLimitFrequency", int(RENDER_HZ))
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        print(f"[Rates] physics={PHYSICS_HZ:g} Hz, render={RENDER_HZ:g} Hz, "
              f"RGB={FRONT_CAMERA_CONFIG['frequency'] if FRONT_CAMERA_CONFIG else 0:g} Hz", flush=True)

        # 3. 초경량 기본 평면 바닥(Ground Plane) 및 완충 물리 물성 설정
        #    - 원리 (충돌 역학 / Rule 2 상세 주석):
        #      restitution=0.0 (완전 비탄성 충돌): 바닥 충돌 에너지를 흡수하여 착륙 시 기체가 위로 튀는 현상(Bouncing) 원천 방지
        #      static_friction=0.8, dynamic_friction=0.6: 접지 시 미끄러짐 방지 및 안정적인 그립 확보
        # print("[Terrain] 충격 흡수 완충 평면(Damped Ground Plane) 생성 중...")
        # self.world.scene.add_default_ground_plane(
        #     static_friction=0.8,
        #     dynamic_friction=0.6,
        #     restitution=0.0
        # )

        # Full Warehouse 환경 로드
        warehouse_path = SIMULATION_ENVIRONMENTS["Full Warehouse"]
        print(f"[Terrain] Loading Full Warehouse: {warehouse_path}", flush=True)
        self.pg.load_asset(warehouse_path, "/World/layout")

        # Pump Kit before creating sensors/resetting physics so referenced assets
        # and textures can finish loading while the window remains responsive.
        import time
        loading_started = time.monotonic()
        next_report = loading_started
        for _ in range(10):
            simulation_app.update()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            simulation_app.update()
            now = time.monotonic()
            if now >= next_report:
                print(f"[Terrain] Loading assets: {omni.usd.get_context().get_stage_loading_status()}", flush=True)
                next_report = now + 5.0
            if now - loading_started > 300:
                raise RuntimeError(f"Full Warehouse asset loading timed out: {warehouse_path}")
        from pxr import Usd, UsdGeom
        layout = self.world.stage.GetPrimAtPath("/World/layout")
        mesh_count = sum(
            1 for prim in Usd.PrimRange(layout)
            if prim.IsA(UsdGeom.Mesh)
        )
        if mesh_count == 0:
            raise RuntimeError(f"Full Warehouse contains no meshes: {warehouse_path}")
        print(f"[Terrain] Full Warehouse loaded ({mesh_count} meshes)", flush=True)

        # 시각적 가시성을 위한 직사광(Distant Light) 생성
        stage = self.world.stage
        if not stage.GetPrimAtPath("/World/defaultDistantLight"):
            prim_utils.create_prim(
                "/World/defaultDistantLight",
                "DistantLight",
                attributes={
                    "inputs:intensity": 3000.0,
                    "inputs:angle": 0.53,
                    "inputs:color": (1.0, 1.0, 1.0)
                }
            )

        # A plane alone cannot constrain horizontal LiDAR odometry.
        # Set PEGASUS_SLAM_SCENE=0 when using an existing structured environment.
        if SLAM_TEST_SCENE:
            from slam_test_scene import add_slam_test_scene
            add_slam_test_scene(stage)

        # 4. ZAV 커스텀 쿼드콥터 스폰 (ZAV_v1 공식 프리셋 클래스 사용)
        #    - 내부적으로 ZAV_v1.usd 자동 바인딩
        #    - PX4 MAVLink Backend (TCP 4560) 자동 장착
        #    - PX4 북쪽(NED North) 정렬을 위한 Yaw=+90도 자동 적용
        print("[Spawn] ZAV_v1 프리셋으로 기체 생성 중...")
        self.drone = ZAV_v1(
            stage_prefix="/World/quadrotor",
            vehicle_id=0,
            init_pos=[0.0, 0.0, 0.10],
            lidar_config=MID360_CONFIG,
            camera_config=FRONT_CAMERA_CONFIG,
            imu_config=MID360_IMU_CONFIG,
        )
        print("[LiDAR] PointCloud2: /drone0/mid360/pointcloud (frame: mid360)")

        # 5. 3인칭 추적 카메라(Chase Camera) 및 마우스 스크롤 줌(Zoom) 설정
        #    - cam_dist_back: 기본 드론 후방 거리 (2.0 m)
        #    - cam_height_up: 기본 드론 상단 높이 오프셋 (0.65 m)
        #    - look_ahead_dist: 드론 전방 시선 타겟 거리 (1.8 m -> 드론은 화면 하단 1/3에 위치하고 전방 2/3 시야 확보)
        #    - look_height_offset: 시선 타겟 높이 오프셋 (0.15 m)
        #    - zoom_scale: 마우스 휠 줌 배율 (기본 1.0, 범위 0.35 ~ 4.5)
        self.cam_dist_back = 2.0
        self.cam_height_up = 0.65
        self.look_ahead_dist = 1.8
        self.look_height_offset = 0.15
        self.zoom_scale = 1.0

        # Omniverse 마우스 스크롤 이벤트 구독 등록
        self._app_window = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._mouse = self._app_window.get_mouse()
        self._mouse_sub = self._input.subscribe_to_mouse_events(self._mouse, self._on_mouse_event)

        # 6. 물리 시뮬레이션 환경 리셋 (Articulation 및 관절 초기화)
        print("[INIT] Resetting world...", flush=True)
        self.world.reset()
        print("=" * 60)
        print("[Ready] Full Warehouse 시뮬레이션 및 3인칭 줌 추적 카메라 준비 완료!")
        print("=" * 60)

    def _on_mouse_event(self, event, *args, **kwargs):
        """
        마우스 휠(Scroll) 입력을 감지하여 3인칭 카메라와 드론 사이의 거리를 실시간 조절합니다.
        - 휠 위로(Scroll Up, y > 0): zoom_scale 감소 -> 드론에 가까워짐 (Zoom In)
        - 휠 아래로(Scroll Down, y < 0): zoom_scale 증가 -> 드론에서 멀어짐 (Zoom Out)
        """
        if event.type == carb.input.MouseEventType.SCROLL:
            scroll_y = event.scroll.y
            if scroll_y > 0:
                self.zoom_scale = max(0.35, self.zoom_scale * 0.88)
            elif scroll_y < 0:
                self.zoom_scale = min(4.5, self.zoom_scale * 1.12)
        return True

    def update_chase_camera(self):
        """
        드론의 후방 상단에서 기체와 전방 시야를 동시에 비추며 따라가는 3인칭 추적 카메라 갱신 함수.

        [수학적 원리 및 좌표계 변환 (Rule 2 상세 주석)]:
        1. 기체 위치 및 자세 획득:
           - p_drone = [x, y, z]^T (ENU 월드 좌표계)
           - q_drone = [qx, qy, qz, qw]^T (ENU 월드 대비 FLU 기체 바디의 회전 쿼터니언)
        2. 롤/피치 흔들림 제거 (Horizontal Projection):
           - 기체가 전후좌우 가감속할 때 발생하는 Roll/Pitch 기울기가 카메라에 전달되면 시야가 출렁거림.
           - 기체 FLU 바디의 정면 단위벡터 e_x = [1, 0, 0]^T를 월드 좌표계로 회전 변환한 뒤:
             f_world = R(q_drone) * [1, 0, 0]^T
           - Z성분을 0으로 투영하고 정규화하여 수평 헤딩 단위벡터 u_horiz = [ux, uy, 0]^T를 추출함.
        3. 마우스 휠 배율(zoom_scale)이 반영된 카메라 위치(Eye) 및 시선 타겟(Target) 산출:
           - p_eye    = p_drone - (d_back * zoom_scale) * u_horiz + [0, 0, h_up * zoom_scale]^T
           - p_target = p_drone + d_look * u_horiz + [0, 0, h_look]^T
        """
        pos = self.drone.state.position
        quat = self.drone.state.attitude

        if pos is None or quat is None or np.linalg.norm(quat) < 1e-6:
            return

        # 기체 정면 벡터 [1, 0, 0]을 월드 좌표계로 변환 후 수평면(XY) 투영
        forward_world = Rotation.from_quat(quat).apply([1.0, 0.0, 0.0])
        horiz_norm = np.hypot(forward_world[0], forward_world[1])

        if horiz_norm > 1e-6:
            u_horiz = np.array([forward_world[0] / horiz_norm, forward_world[1] / horiz_norm, 0.0])
        else:
            u_horiz = np.array([0.0, 1.0, 0.0])

        # 줌 배율을 적용한 후방 상단 카메라 위치 및 전방 시선 타겟 계산
        dist_back = self.cam_dist_back * self.zoom_scale
        height_up = self.cam_height_up * self.zoom_scale
        cam_eye = pos - dist_back * u_horiz + np.array([0.0, 0.0, height_up])
        cam_target = pos + self.look_ahead_dist * u_horiz + np.array([0.0, 0.0, self.look_height_offset])

        self.pg.set_viewport_camera(cam_eye.tolist(), cam_target.tolist())

    def run(self):
        """시뮬레이션 메인 루프 (물리 스텝 및 렌더링 갱신)"""
        self.timeline.play()
        while simulation_app.is_running():
            self.world.step(render=True)
            self.update_chase_camera()
        
        self.timeline.stop()
        simulation_app.close()

if __name__ == '__main__':
    app = ZAVPlaneApp()
    app.run()
