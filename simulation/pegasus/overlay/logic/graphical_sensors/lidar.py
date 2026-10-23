"""
| File: lidar.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Simulates a lidar attached to the vehicle
"""
__all__ = ["Lidar"]

from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.graphical_sensors import GraphicalSensor
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Imports the python bindings to interact with lidar sensor
import omni.kit.commands
from pxr import Gf, Sdf, UsdGeom
from omni.usd import get_stage_next_free_path

# Auxiliary scipy and numpy modules
import numpy as np
from scipy.spatial.transform import Rotation

# Import the replicatore core module used for writing graphical data to ROS 2
import omni.replicator.core as rep

class Lidar(GraphicalSensor):

    def __init__(self, lidar_name, config={}):
        """
        Initialize the Lidar class

        The available configurations for the lidar are available in the folder exts/omni.isaac.sensor/data/lidar_configs
        Check also the official documentation:
        https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_rtx_based_lidar.html
        """

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="Lidar", update_rate=config.get("frequency", 60.0)) 

        # Setup the name of the camera primitive path
        self._lidar_name = lidar_name
        self._stage_prim_path = ""

        # Configurations of the Lidar
        self._position = config.get("position", np.array([0.0, 0.0, 0.10]))
        self._orientation = Rotation.from_euler("ZYX", config.get("orientation", np.array([0.0, 0.0, 0.0])), degrees=True).as_quat()
        self._sensor_configuration = config.get("sensor_configuration", "Example_Rotary")
        self._show_render = config.get("show_render", False)
        self._publish_laserscan = config.get("publish_laserscan", True)
        self._publish_method = config.get("publish_method", "writer")
        self._tcp_port = config.get("tcp_port")
        self._livox_tcp_port = config.get("livox_tcp_port")
        self._livox_scan_lines = config.get("livox_scan_lines", 4)
        if self._publish_method not in ("writer", "python"):
            raise ValueError("publish_method must be 'writer' or 'python'")
        self._usd_path = config.get("usd_path")
        self._sensor_prim_path = config.get(
            "sensor_prim_path", "/World/Example_Rotary"
        )
        self._sensor = None

    def initialize(self, vehicle):
        """
        Initialize the Lidar sensor
        """
        super().initialize(vehicle)
        
        # Get the complete stage prefix for the lidar
        self._stage_prim_path = get_stage_next_free_path(PegasusInterface().world.stage, self._vehicle.prim_path + "/body/" + self._lidar_name, False)

        # Get the camera name that was actually created (and update the camera name)
        self._lidar_name = self._stage_prim_path.rpartition("/")[-1]

        # [ZAV 변경] 저장된 USD의 센서 subtree만 참조해 body 아래에 장착합니다.
        if self._usd_path:
            # Reference only the sensor subtree, excluding saved lights/render settings.
            stage = PegasusInterface().world.stage
            mount = UsdGeom.Xform.Define(stage, self._stage_prim_path)
            mount.AddTranslateOp().Set(Gf.Vec3d(*map(float, self._position)))
            mount.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(
                float(self._orientation[3]),
                Gf.Vec3d(*map(float, self._orientation[:3])),
            ))
            self._stage_prim_path += "/sensor"
            self._sensor = stage.DefinePrim(self._stage_prim_path)
            self._sensor.GetReferences().AddReference(
                self._usd_path, Sdf.Path(self._sensor_prim_path)
            )
            if self._sensor.GetTypeName() != "OmniLidar":
                raise RuntimeError(
                    f"Expected OmniLidar at {self._stage_prim_path}; "
                    f"check {self._usd_path}:{self._sensor_prim_path}"
                )
            return
        
        _, self._sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=self._lidar_name,
            parent=self._vehicle.prim_path + "/body",
            config=self._sensor_configuration,
            translation=(self._position[0], self._position[1], self._position[2]),
            orientation=Gf.Quatd(self._orientation[3], self._orientation[0], self._orientation[1], self._orientation[2])
        )
    
    def start(self):

        # If show_render is True, then create a render product for the lidar in the Isaac Sim environment
        if self._show_render:
            hydra_texture = rep.create.render_product(self._sensor.GetPath(), [1, 1], name="Isaac")
            writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
            writer.initialize(topicName="point_cloud", frameId="base_scan")
            writer.attach([hydra_texture])

    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state
    
    @GraphicalSensor.update_at_rate
    def update(self, state: State, dt: float):
        """Method that gets the data from the lidar and returns it as a dictionary.

        Args:
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """

        # Just return the prim path and the name of the lidar
        self._state = {
            "lidar_name": self._lidar_name,
            "stage_prim_path": self._stage_prim_path,
            "publish_laserscan": self._publish_laserscan,
            "publish_method": self._publish_method,
            "tcp_port": self._tcp_port,
            "livox_tcp_port": self._livox_tcp_port,
            "livox_scan_lines": self._livox_scan_lines,
        }

        return self._state
