# [ZAV 변경] 실센서 YAML은 유지하고 시뮬레이터 센서의 장착 위치/광학계를 역산합니다.
"""Derive simulator mounts from the unchanged FAST-LIVO calibration YAMLs."""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def load_calibration(config_dir, lidar_position, lidar_orientation):
    config_dir = Path(config_dir)
    params = yaml.safe_load((config_dir / "zenith.yaml").read_text())["/laserMapping"]["ros__parameters"]
    camera = yaml.safe_load((config_dir / "zenith_camera_pinhole.yaml").read_text())["/laserMapping"]["ros__parameters"]["camera"]
    ext = params["extrin_calib"]
    r_bl = Rotation.from_euler("ZYX", lidar_orientation, degrees=True).as_matrix()
    r_cl = Rotation.from_matrix(np.asarray(ext["Rcl"]).reshape(3, 3)).as_matrix()
    r_il = Rotation.from_matrix(np.asarray(ext["extrinsic_R"]).reshape(3, 3)).as_matrix()
    # body FLU -> optical RDF for a forward-facing Camera with camera_axes=world.
    r_optical_from_flu = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
    r_boptical = r_bl @ r_cl.T
    r_bcamera = r_boptical @ r_optical_from_flu
    r_bimu = r_bl @ r_il.T
    p_bl = np.asarray(lidar_position, dtype=float)
    return {
        "camera_position": (p_bl - r_boptical @ np.asarray(ext["Pcl"])).tolist(),
        "camera_orientation": Rotation.from_matrix(r_bcamera).as_euler("ZYX", degrees=True).tolist(),
        "imu_position": (p_bl - r_bimu @ np.asarray(ext["extrinsic_T"])).tolist(),
        "imu_orientation": Rotation.from_matrix(r_bimu).as_euler("ZYX", degrees=True).tolist(),
        "resolution": (int(camera["cam_width"]), int(camera["cam_height"])),
        "intrinsics": [[camera["cam_fx"], 0., camera["cam_cx"]],
                       [0., camera["cam_fy"], camera["cam_cy"]], [0., 0., 1.]],
        "distortion": [camera[f"cam_d{i}"] for i in range(4)] + [0.] * 8,
        "scan_lines": int(params["preprocess"]["scan_line"]),
    }
