"""
| File: zav_v1.py
| Description: ZAV Quadrotor Custom Vehicle class inheriting from Multirotor
| Vehicle Specs:
|   - Mass: 1.99 kg
|   - Arm length: 140.0 mm (Wheelbase 280 mm, dx = +/-99 mm, dy = +/-99 mm)
|   - Backend: PX4 MAVLink Backend (Lockstep enabled)
"""
__all__ = ["ZAV_v1"]

import os
import carb
from scipy.spatial.transform import Rotation

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig


class ZAV_v1(Multirotor):
    """
    ZAV Custom Quadcopter simulation class.
    Automatically binds ZAV_v1.usd, standard sensors, and PX4 MAVLink backend.
    """

    def __init__(
        self,
        stage_prefix: str = "/World/quadrotor",
        vehicle_id: int = 0,
        init_pos=[0.0, 0.0, 0.10],
        init_orientation=None,
        config=None,
        lidar_config=None,
        camera_config=None,
        imu_config=None,
    ):
        # 1. USD 에셋 경로 확인
        if "ZAV_v1" not in ROBOTS or not os.path.exists(ROBOTS["ZAV_v1"]):
            carb.log_error(f"❌ [에러] ZAV_v1 드론 에셋을 찾을 수 없습니다! 경로: {ROBOTS.get('ZAV_v1')}")
            raise FileNotFoundError(f"ZAV_v1 asset not found at {ROBOTS.get('ZAV_v1')}")
        usd_path = ROBOTS["ZAV_v1"]

        # 2. 초기 각도 설정: 미지정 시 PX4 북쪽(+Y)과 일치하도록 Yaw=+90도 기본 적용
        if init_orientation is None:
            init_orientation = Rotation.from_euler("XYZ", [0.0, 0.0, 90.0], degrees=True).as_quat()

        # 3. 드론 통신 백엔드, 센서 및 액추에이터 동역학 설정
        if config is None:
            config = MultirotorConfig()

            # -------------------------------------------------------------
            # [액추에이터 동역학: 추력 및 반작용 토크 계수]
            # 기본 템플릿(Iris 3S/920KV/10인치 프롭) 기준:
            #   - rotor_constant: [8.54858e-06, 8.54858e-06, 8.54858e-06, 8.54858e-06]
            #   - rolling_moment_coefficient: [1e-06, 1e-06, 1e-06, 1e-06]
            #   - max_rotor_velocity: [1100.0, 1100.0, 1100.0, 1100.0]
            #
            # ZAV v1 실측치 기준 (T-Motor F90 1500KV + HQ 70403 7인치 프롭, 6S LiPo):
            #   - 100% 스로틀: F_max = 20.62 N (2,102.78 g), RPM_max = 18,501 RPM (omega_max = 1937.4 rad/s)
            #   - C_T (rotor_constant) = 20.62 / (1937.4)^2 ≈ 5.4935e-06 N/(rad/s)^2
            #   - C_M (rolling_moment) = C_T * 0.018 ≈ 9.888e-08 N*m/(rad/s)^2
            # -------------------------------------------------------------
            from pegasus.simulator.logic.thrusters import QuadraticThrustCurve

            config.thrust_curve = QuadraticThrustCurve({
                "num_rotors": 4,
                # 기본값 보존 (Iris 템플릿):
                # "rotor_constant": [8.54858e-06, 8.54858e-06, 8.54858e-06, 8.54858e-06],
                # "rolling_moment_coefficient": [1e-06, 1e-06, 1e-06, 1e-06],
                # "max_rotor_velocity": [1100.0, 1100.0, 1100.0, 1100.0],

                # ZAV v1 실측값 적용:
                "rotor_constant": [5.4935e-06, 5.4935e-06, 5.4935e-06, 5.4935e-06],
                "rolling_moment_coefficient": [9.888e-08, 9.888e-08, 9.888e-08, 9.888e-08],
                "rot_dir": [-1, -1, 1, 1],
                "min_rotor_velocity": [0.0, 0.0, 0.0, 0.0],
                "max_rotor_velocity": [2000.0, 2000.0, 2000.0, 2000.0],
            })

            # -------------------------------------------------------------
            # [PX4 MAVLink 백엔드 제어/스케일링 파라미터]
            # 기본 템플릿 기준:
            #   - input_scaling: [1000.0, 1000.0, 1000.0, 1000.0]
            #   - zero_position_armed: [100.0, 100.0, 100.0, 100.0]
            #
            # ZAV v1 실측치 기준:
            #   - input_scaling (최대 각속도): 1937.4 rad/s (18,501 RPM)
            #   - zero_position_armed (시동 아이들링 약 8%): 155.0 rad/s (~1480 RPM)
            # -------------------------------------------------------------
            mavlink_config = PX4MavlinkBackendConfig({
                "vehicle_id": vehicle_id,
                # [ZAV 변경] 기본 localhost를 유지하고 WSL NAT 등 별도 host 설정만 허용합니다.
                "connection_ip": os.environ.get("PEGASUS_BIND_HOST", "localhost"),
                "px4_autolaunch": False,
                "px4_vehicle_model": "isaac_zav",

                # 기본값 보존:
                # "input_offset": [0.0, 0.0, 0.0, 0.0],
                # "input_scaling": [1000.0, 1000.0, 1000.0, 1000.0],
                # "zero_position_armed": [100.0, 100.0, 100.0, 100.0],

                # ZAV v1 실측값 적용:
                "input_offset": [0.0, 0.0, 0.0, 0.0],
                "input_scaling": [1937.4, 1937.4, 1937.4, 1937.4],
                "zero_position_armed": [155.0, 155.0, 155.0, 155.0],
                "update_rate": 250.0,
            })
            config.backends = [PX4MavlinkBackend(mavlink_config)]

        # [ZAV 변경] 기존 공력/PX4 Backend는 그대로 두고 선택한 센서만 추가합니다.
        # Optional USD LiDAR; keep the existing flight configuration and PX4 backend.
        if imu_config is not None:
            from pegasus.simulator.logic.sensors.mounted_imu import MountedIMU
            config.sensors.append(MountedIMU(imu_config))

        if lidar_config is not None or camera_config is not None or imu_config is not None:
            from pegasus.simulator.logic.graphical_sensors.lidar import Lidar
            from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend

            if lidar_config is not None:
                config.graphical_sensors.append(Lidar("mid360", config=lidar_config))
            if camera_config is not None:
                from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
                config.graphical_sensors.append(MonocularCamera("front_camera", config=camera_config))
            if not any(isinstance(backend, ROS2Backend) for backend in config.backends):
                config.backends.append(ROS2Backend(
                    vehicle_id=vehicle_id,
                    config={
                        "namespace": "drone",
                        "pub_graphical_sensors": True,
                        "pub_sensors": False,
                        "pub_state": os.environ.get("ZAV_DIAGNOSTIC_STATE", "0") == "1",
                        "sub_control": False,
                    },
                ))

        # 4. 부모 Multirotor 초기화 (드론 스폰 및 관절/물리 바인딩 완료)
        super().__init__(
            stage_prefix=stage_prefix,
            usd_file=usd_path,
            vehicle_id=vehicle_id,
            init_pos=init_pos,
            init_orientation=init_orientation,
            config=config,
        )
