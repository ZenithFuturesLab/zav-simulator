"""
| File: ros2_backend.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: File that implements the ROS2 Backend for communication/control with/of the vehicle simulation through ROS2 topics
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
"""

# Make sure the ROS2 extension is enabled
import carb
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")

# ROS2 imports
import rclpy
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus, PointCloud2, PointField, Image
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
import numpy as np
from array import array
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped

# TF imports
# Check if these libraries exist in the system
try:
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from tf2_ros.transform_broadcaster import TransformBroadcaster
    tf2_ros_loaded = True
except ImportError:
    carb.log_warn("TF2 ROS not installed. Will not publish TFs with the ROS2 backend")
    tf2_ros_loaded = False

from pegasus.simulator.logic.backends.backend import Backend

# Import the replicatore core module used for writing graphical data to ROS 2
import omni
import omni.graph.core as og
import omni.replicator.core as rep
from isaacsim.ros2.bridge import read_camera_info


class ROS2Backend(Backend):

    def __init__(self, vehicle_id: int, num_rotors=4, config: dict = {}):
        """Initialize the ROS2 Camera class

        Args:
            camera_prim_path (str): Path to the camera prim. Global path when it starts with `/`, else local to vehicle prim path
            config (dict): A Dictionary that contains all the parameters for configuring the ROS2Camera - it can be empty or only have some of the parameters used by the ROS2Camera.

        Examples:
            The dictionary default parameters are

            >>> {"namespace": "drone"                           # Namespace to append to the topics
            >>>  "pub_pose": True,                              # Publish the pose of the vehicle
            >>>  "pub_twist": True,                             # Publish the twist of the vehicle
            >>>  "pub_twist_inertial": True,                    # Publish the twist of the vehicle in the inertial frame
            >>>  "pub_accel": True,                             # Publish the acceleration of the vehicle
            >>>  "pub_imu": True,                               # Publish the IMU data
            >>>  "pub_mag": True,                               # Publish the magnetometer data
            >>>  "pub_gps": True,                               # Publish the GPS data
            >>>  "pub_gps_vel": True,                           # Publish the GPS velocity data
            >>>  "pose_topic": "state/pose",                    # Position and attitude of the vehicle in ENU
            >>>  "twist_topic": "state/twist",                  # Linear and angular velocities in the body frame of the vehicle
            >>>  "twist_inertial_topic": "state/twist_inertial" # Linear velocity of the vehicle in the inertial frame
            >>>  "accel_topic": "state/accel",                  # Linear acceleration of the vehicle in the inertial frame
            >>>  "imu_topic": "sensors/imu",                    # IMU data
            >>>  "mag_topic": "sensors/mag",                    # Magnetometer data
            >>>  "gps_topic": "sensors/gps",                    # GPS data
            >>>  "gps_vel_topic": "sensors/gps_twist",          # GPS velocity data
            >>>  "pub_graphical_sensors": True,                 # Publish the graphical sensors
            >>>  "pub_sensors": True,                           # Publish the sensors
            >>>  "pub_state": True,                             # Publish the state of the vehicle
            >>>  "pub_tf": False,                               # Publish the TF of the vehicle
            >>>  "sub_control": True,                           # Subscribe to the control topics
        """

        # Save the configurations for this backend
        self._id = vehicle_id
        self._num_rotors = num_rotors
        self._namespace = config.get("namespace", "drone" + str(vehicle_id))

        # Save what whould be published/subscribed
        self._pub_graphical_sensors = config.get("pub_graphical_sensors", True)
        self._pub_sensors = config.get("pub_sensors", True)
        self._pub_state = config.get("pub_state", True)
        self._sub_control = config.get("sub_control", True)

        # Check if the tf2_ros library is loaded and if the flag is set to True
        self._pub_tf = config.get("pub_tf", False) and tf2_ros_loaded

        # Start the actual ROS2 setup here
        try:
            rclpy.init()
        except:
            # If rclpy is already initialized, just ignore the exception
            pass

        self.node = rclpy.create_node("simulator_vehicle_" + str(vehicle_id))

        # Initialize the publishers and subscribers
        self.initialize_publishers(config)
        self.initialize_subscribers()

        # Create a dictionary that will store the writers for the graphical sensors
        # NOTE: this is done this way, because the writers move data from the GPU->CPU and then publish it to ROS2
        # in a separate thread. This is done to avoid blocking the simulation
        self.graphical_sensors_writers = {}
        self._python_lidars = {}
        self._lidar_first_cloud_logged = set()
        self._lidar_streams = {}
        self._python_cameras = {}
        self._mounted_imu_publishers = {}
        self._livox_packetizers = {}
        self._lidar_gmo = {}
        self._lidar_clock_offsets = {}
        
        # Setup zero input reference for the thrusters
        self.input_ref = [0.0 for i in range(self._num_rotors)]

        # -----------------------------------------------------
        # Initialize the static and dynamic tf broadcasters
        # -----------------------------------------------------
        if self._pub_tf:

            # Initiliaze the static tf broadcaster for the sensors
            self.tf_static_broadcaster = StaticTransformBroadcaster(self.node)

            # Initialize the static tf broadcaster for the base_link transformation
            self.send_static_transforms()

            # Initialize the dynamic tf broadcaster for the position of the body of the vehicle (base_link) with respect to the inertial frame (map - ENU) expressed in the inertil frame (map - ENU)
            self.tf_broadcaster = TransformBroadcaster(self.node)
    
    
    def initialize_publishers(self, config: dict):

        # ----------------------------------------------------- 
        # Create publishers for the state of the vehicle in ENU
        # -----------------------------------------------------
        if self._pub_state:
            if config.get("pub_pose", True):
                self.pose_pub = self.node.create_publisher(PoseStamped, self._namespace + str(self._id) + "/" + config.get("pose_topic", "state/pose"), rclpy.qos.qos_profile_sensor_data)
            
            if config.get("pub_twist", True):
                self.twist_pub = self.node.create_publisher(TwistStamped, self._namespace + str(self._id) + "/" + config.get("twist_topic", "state/twist"), rclpy.qos.qos_profile_sensor_data)

            if config.get("pub_twist_inertial", True):
                self.twist_inertial_pub = self.node.create_publisher(TwistStamped, self._namespace + str(self._id) + "/" + config.get("twist_inertial_topic", "state/twist_inertial"), rclpy.qos.qos_profile_sensor_data)

            if config.get("pub_accel", True):
                self.accel_pub = self.node.create_publisher(AccelStamped, self._namespace + str(self._id) + "/" + config.get("accel_topic", "state/accel"), rclpy.qos.qos_profile_sensor_data)

        # -----------------------------------------------------
        # Create publishers for the sensors of the vehicle
        # -----------------------------------------------------
        if self._pub_sensors:
            if config.get("pub_imu", True):
                self.imu_pub = self.node.create_publisher(Imu, self._namespace + str(self._id) + "/" + config.get("imu_topic", "sensors/imu"), rclpy.qos.qos_profile_sensor_data)
            
            if config.get("pub_mag", True):
                self.mag_pub = self.node.create_publisher(MagneticField, self._namespace + str(self._id) + "/" + config.get("mag_topic", "sensors/mag"), rclpy.qos.qos_profile_sensor_data)

            if config.get("pub_gps", True):
                self.gps_pub = self.node.create_publisher(NavSatFix, self._namespace + str(self._id) + "/" + config.get("gps_topic", "sensors/gps"), rclpy.qos.qos_profile_sensor_data)
            
            if config.get("pub_gps_vel", True):
                self.gps_vel_pub = self.node.create_publisher(TwistStamped, self._namespace + str(self._id) + "/" + config.get("gps_vel_topic", "sensors/gps_twist"), rclpy.qos.qos_profile_sensor_data)
        

    def initialize_subscribers(self):

        if self._sub_control:
            # Subscribe to vector of floats with the target angular velocities to control the vehicle
            # This is not ideal, but we need to reach out to NVIDIA so that they can improve the ROS2 support with custom messages
            # The current setup as it is.... its a pain!!!!
            self.rotor_subs = []
            for i in range(self._num_rotors):
                self.rotor_subs.append(self.node.create_subscription(Float64, self._namespace + str(self._id) + "/control/rotor" + str(i) + "/ref", lambda x: self.rotor_callback(x, i),10))


    def send_static_transforms(self):

        # Create the transformation from base_link FLU (ROS standard) to base_link FRD (standard in airborn and marine vehicles)
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = self._namespace + '_' + 'base_link'
        t.child_frame_id = self._namespace + '_' + 'base_link_frd'

        # Converts from FLU to FRD
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 1.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0

        self.tf_static_broadcaster.sendTransform(t)

        # Create the transform from map, i.e inertial frame (ROS standard) to map_ned (standard in airborn or marine vehicles)
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "map_ned"
        
        # Converts ENU to NED
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = -0.7071068
        t.transform.rotation.y = -0.7071068
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0

        self.tf_static_broadcaster.sendTransform(t)

    def update_state(self, state):
        """
        Method that when implemented, should handle the receivel of the state of the vehicle using this callback
        """

        # Publish the state of the vehicle only if the flag is set to True
        if not self._pub_state:
            return

        pose = PoseStamped()
        twist = TwistStamped()
        twist_inertial = TwistStamped()
        accel = AccelStamped()

        # Update the header
        pose.header.stamp = self.node.get_clock().now().to_msg()
        twist.header.stamp = pose.header.stamp
        twist_inertial.header.stamp = pose.header.stamp
        accel.header.stamp = pose.header.stamp

        pose.header.frame_id = "map"
        twist.header.frame_id = self._namespace + "_" + "base_link"
        twist_inertial.header.frame_id = "map"
        accel.header.frame_id = "map"

        # Fill the position and attitude of the vehicle in ENU
        pose.pose.position.x = state.position[0]
        pose.pose.position.y = state.position[1]
        pose.pose.position.z = state.position[2]

        pose.pose.orientation.x = state.attitude[0]
        pose.pose.orientation.y = state.attitude[1]
        pose.pose.orientation.z = state.attitude[2]
        pose.pose.orientation.w = state.attitude[3]

        # Fill the linear and angular velocities in the body frame of the vehicle
        twist.twist.linear.x = state.linear_body_velocity[0]
        twist.twist.linear.y = state.linear_body_velocity[1]
        twist.twist.linear.z = state.linear_body_velocity[2]

        twist.twist.angular.x = state.angular_velocity[0]
        twist.twist.angular.y = state.angular_velocity[1]
        twist.twist.angular.z = state.angular_velocity[2]

        # Fill the linear velocity of the vehicle in the inertial frame
        twist_inertial.twist.linear.x = state.linear_velocity[0]
        twist_inertial.twist.linear.y = state.linear_velocity[1]
        twist_inertial.twist.linear.z = state.linear_velocity[2]

        # Fill the linear acceleration in the inertial frame
        accel.accel.linear.x = state.linear_acceleration[0]
        accel.accel.linear.y = state.linear_acceleration[1]
        accel.accel.linear.z = state.linear_acceleration[2]

        # Publish the messages containing the state of the vehicle
        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)
        self.twist_inertial_pub.publish(twist_inertial)
        self.accel_pub.publish(accel)

        # Update the dynamic tf broadcaster with the current position of the vehicle in the inertial frame
        if self._pub_tf:
            t = TransformStamped()
            t.header.stamp = pose.header.stamp
            t.header.frame_id = "map"
            t.child_frame_id = self._namespace + '_' + 'base_link'
            t.transform.translation.x = state.position[0]
            t.transform.translation.y = state.position[1]
            t.transform.translation.z = state.position[2]
            t.transform.rotation.x = state.attitude[0]
            t.transform.rotation.y = state.attitude[1]
            t.transform.rotation.z = state.attitude[2]
            t.transform.rotation.w = state.attitude[3]
            self.tf_broadcaster.sendTransform(t)
        

    def rotor_callback(self, ros_msg: Float64, rotor_id):
        # Update the reference for the rotor of the vehicle
        self.input_ref[rotor_id] = float(ros_msg.data)

    def update_sensor(self, sensor_type: str, data):
        """
        Method that when implemented, should handle the receival of sensor data
        """

        # An explicitly installed mounted IMU is independent of standard PX4 sensors.
        if sensor_type == "MountedIMU":
            self._publish_mounted_imu(data)
            return

        # Only process sensor data if the flag is set to True
        if not self._pub_sensors:
            return

        if sensor_type == "IMU":
            self.update_imu_data(data)
        elif sensor_type == "GPS":
            self.update_gps_data(data)
        elif sensor_type == "Magnetometer":
            self.update_mag_data(data)
        else:
            pass

    def update_graphical_sensor(self, sensor_type: str, data):
        """
        Method that when implemented, should handle the receival of graphical sensor data
        """

        # Only process graphical sensor data if the flag is set to True
        if not self._pub_graphical_sensors:
            return

        if sensor_type == "MonocularCamera":
            self.update_monocular_camera_data(data)
        elif sensor_type == "Lidar":
            self.update_lidar_data(data)
        else:
            pass

    # [ZAV 변경] PX4 기본 IMU와 별도로 장착 원점의 가속도/각속도를 같은 시뮬레이션 시각으로 전송합니다.
    def _publish_mounted_imu(self, data):
        config = data["config"]
        topic = f"/{self._namespace}{self._id}/{config.get('topic', 'mid360/imu')}"
        if topic not in self._mounted_imu_publishers:
            # [ZAV 변경] TCP 모드에서는 외부 수신기만 ROS에 발행합니다. Ubuntu 중복 발행을 방지합니다.
            self._mounted_imu_publishers[topic] = None if config.get("tcp_port") else self.node.create_publisher(
                Imu, topic, QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT)
            )
            print(f"[IMU] {topic}, frame={config.get('frame_id', 'mid360')}", flush=True)
        msg = Imu()
        stamp_ns = round(float(self._vehicle._world.current_time) * 1_000_000_000)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        msg.header.frame_id = config.get("frame_id", "mid360")
        # Raw gyro/accelerometer data: no attitude estimator is simulated.
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0
        for i, axis in enumerate(("x", "y", "z")):
            setattr(msg.angular_velocity, axis, float(data["angular_velocity"][i]))
            setattr(msg.linear_acceleration, axis, float(data["linear_acceleration"][i]))
            msg.angular_velocity_covariance[i * 4] = float(data["gyro_variance"])
            msg.linear_acceleration_covariance[i * 4] = float(data["accel_variance"])
        if self._mounted_imu_publishers[topic] is not None:
            self._mounted_imu_publishers[topic].publish(msg)
        if config.get("tcp_port"):
            key = "imu:" + topic
            if key not in self._lidar_streams:
                from .pointcloud_stream import PointCloudStream
                self._lidar_streams[key] = PointCloudStream(int(config["tcp_port"]), capacity=256)
            self._lidar_streams[key].submit(serialize_message(msg))

    def update_imu_data(self, data):

        msg = Imu()

        # Update the header
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self._namespace + '_' + "base_link_frd"
        
        # Update the angular velocity (NED + FRD)
        msg.angular_velocity.x = data["angular_velocity"][0]
        msg.angular_velocity.y = data["angular_velocity"][1]
        msg.angular_velocity.z = data["angular_velocity"][2]
        
        # Update the linear acceleration (NED)
        msg.linear_acceleration.x = data["linear_acceleration"][0]
        msg.linear_acceleration.y = data["linear_acceleration"][1]
        msg.linear_acceleration.z = data["linear_acceleration"][2]

        # Publish the message with the current imu state
        self.imu_pub.publish(msg)

    def update_gps_data(self, data):

        msg = NavSatFix()
        msg_vel = TwistStamped()

        # Update the headers
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map_ned"
        msg_vel.header.stamp = msg.header.stamp
        msg_vel.header.frame_id = msg.header.frame_id

        # Update the status of the GPS
        status_msg = NavSatStatus()
        status_msg.status = 0 # unaugmented fix position
        status_msg.service = 1 # GPS service
        msg.status = status_msg

        # Update the latitude, longitude and altitude
        msg.latitude = data["latitude"]
        msg.longitude = data["longitude"]
        msg.altitude = data["altitude"]

        # Update the velocity of the vehicle measured by the GPS in the inertial frame (NED)
        msg_vel.twist.linear.x = data["velocity_north"]
        msg_vel.twist.linear.y = data["velocity_east"]
        msg_vel.twist.linear.z = data["velocity_down"]

        # Publish the message with the current GPS state
        self.gps_pub.publish(msg)
        self.gps_vel_pub.publish(msg_vel)

    def update_mag_data(self, data):
        
        msg = MagneticField()

        # Update the headers
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link_frd"

        msg.magnetic_field.x = data["magnetic_field"][0]
        msg.magnetic_field.y = data["magnetic_field"][1]
        msg.magnetic_field.z = data["magnetic_field"][2]

        # Publish the message with the current magnetic data
        self.mag_pub.publish(msg)

    def update_monocular_camera_data(self, data):

        if data.get("publish_method", "writer") == "python":
            self._publish_python_camera(data)
            return

        # Check if the camera name exists in the writers dictionary
        if data["camera_name"] not in self.graphical_sensors_writers:
            self.add_monocular_camera_writter(data)

    def _publish_python_camera(self, data):
        name = data["camera_name"]
        rgba = data["camera"].get_rgba()
        if rgba is None or np.asarray(rgba).size == 0:
            return
        rgba = np.asarray(rgba)
        if rgba.ndim != 3 or rgba.shape[2] < 3:
            return
        if name not in self._python_cameras:
            topic = f"/{self._namespace}{self._id}/{name}/color/image_raw"
            # [ZAV 변경] TCP 사용 시 직접 DDS 발행을 생략하고 같은 CDR 메시지만 보냅니다.
            self._python_cameras[name] = None if data.get("tcp_port") else self.node.create_publisher(
                Image, topic, qos_profile_sensor_data
            )
            print(f"[Camera] RGB publisher: {topic}", flush=True)
        rgb = np.ascontiguousarray(rgba[:, :, :3], dtype=np.uint8)
        msg = Image()
        msg.header.frame_id = name + "_optical"
        stamp_ns = round(float(self._vehicle._world.current_time) * 1_000_000_000)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = msg.width * 3
        # The generated setter checks bytes one by one; array('B') takes its
        # supported bulk-buffer path instead of blocking the render callback.
        msg.data = array('B', rgb.tobytes())
        if self._python_cameras[name] is not None:
            self._python_cameras[name].publish(msg)
        if data.get("tcp_port"):
            stream_key = "camera:" + name
            if stream_key not in self._lidar_streams:
                from .pointcloud_stream import PointCloudStream
                self._lidar_streams[stream_key] = PointCloudStream(int(data["tcp_port"]))
            self._lidar_streams[stream_key].submit(serialize_message(msg))


    def add_monocular_camera_writter(self, data):

        # List all the available writers: print(rep.writers.WriterRegistry._writers)
        render_prod_path = data["camera"]._render_product_path

        # Create the writer for the rgb camera
        writer = rep.writers.get("LdrColorSDROS2PublishImage")
        writer.initialize(nodeNamespace=self._namespace + str(self._id), topicName=data["camera_name"] + "/color/image_raw", frameId=data["camera_name"], queueSize=1)
        writer.attach([render_prod_path])

        # Add the writer to the dictionary
        self.graphical_sensors_writers[data["camera_name"]] = [writer]

        # Check if depth is enabled, if so, set the depth properties
        if "depth" in data:

            # Create the writer for the depth camera
            writer_depth = rep.writers.get("DistanceToImagePlaneSDROS2PublishImage")
            writer_depth.initialize(nodeNamespace=self._namespace + str(self._id), topicName=data["camera_name"] + "/depth", frameId=data["camera_name"], queueSize=1)
            writer_depth.attach([render_prod_path])

            # Add the writer to the dictionary
            self.graphical_sensors_writers[data["camera_name"]].append(writer_depth)

        # Create a writer for publishing the camera info
        writer_info = rep.writers.get("ROS2PublishCameraInfo")
        camera_info, _ = read_camera_info(render_product_path=render_prod_path)
        writer_info.initialize(
            nodeNamespace=self._namespace + str(self._id), 
            topicName=data["camera_name"] + "/color/camera_info", 
            frameId=data["camera_name"], 
            queueSize=1,
            width=camera_info.width,
            height=camera_info.height,
            projectionType=camera_info.distortion_model,
            k=camera_info.k.reshape([1, 9]),
            r=camera_info.r.reshape([1, 9]),
            p=camera_info.p.reshape([1, 12]),
            physicalDistortionModel=camera_info.distortion_model,
            physicalDistortionCoefficients=camera_info.d,
        )

        writer_info.attach([render_prod_path])

        # Add the writer to the dictionary
        self.graphical_sensors_writers[data["camera_name"]].append(writer_info)

        gate_path = omni.syntheticdata.SyntheticData._get_node_path("PostProcessDispatch" + "IsaacSimulationGate", render_prod_path)

        # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
        og.Controller.attribute(gate_path + ".inputs:step").set(int(60/data["frequency"]))

    def update_lidar_data(self, data):

        if data.get("publish_method", "writer") == "python":
            self._publish_python_lidar(data)
            return

        # Check if the lidar name exists in the writers dictionary
        if data["lidar_name"] not in self.graphical_sensors_writers:
            self.add_lidar_writter(data)

    # [ZAV 변경] XYZ 미리보기와 점별 시각을 가진 Livox 호환 패킷을 같은 RTX 측정에서 만듭니다.
    def _publish_python_lidar(self, data):
        """Read the RTX annotator on the render callback and publish XYZ on ROS 2."""
        name = data["lidar_name"]
        if name not in self._python_lidars:
            render_product = rep.create.render_product(
                data["stage_prim_path"], [1, 1], name=name
            )
            annotator = rep.AnnotatorRegistry.get_annotator(
                "IsaacExtractRTXSensorPointCloudNoAccumulator"
            )
            if data.get("livox_tcp_port"):
                annotator.initialize(outputTimestamp=True, outputEmitterId=True, outputIntensity=True)
                gmo_annotator = rep.AnnotatorRegistry.get_annotator("GenericModelOutput")
                gmo_annotator.attach([render_product])
                self._lidar_gmo[name] = gmo_annotator
            annotator.attach([render_product])
            topic = f"/{self._namespace}{self._id}/{name}/pointcloud"
            # [ZAV 변경] 기존 writer/direct DDS 경로는 유지하되 TCP와 동시 발행하지 않습니다.
            publisher = None if data.get("tcp_port") else self.node.create_publisher(PointCloud2, topic, qos_profile_sensor_data)
            self._python_lidars[name] = (render_product, annotator, publisher)
            print(f"[LiDAR] Python PointCloud2 publisher: {topic}", flush=True)

        _, annotator, publisher = self._python_lidars[name]
        result = annotator.get_data()
        points = result.get("data", []) if isinstance(result, dict) else result
        if points is None or len(points) == 0:
            return
        xyz = np.asarray(points, dtype="<f4").reshape(-1, 3)
        msg = PointCloud2()
        msg.header.frame_id = name
        sim_time = float(self._vehicle._world.current_time)
        stamp_ns = round(sim_time * 1_000_000_000)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        msg.height = 1
        msg.width = len(xyz)
        msg.fields = [
            PointField(name=axis, offset=i * 4, datatype=PointField.FLOAT32, count=1)
            for i, axis in enumerate(("x", "y", "z"))
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = False
        msg.data = array('B', xyz.tobytes())
        if publisher is not None:
            publisher.publish(msg)
        if data.get("tcp_port"):
            if name not in self._lidar_streams:
                from .pointcloud_stream import PointCloudStream
                self._lidar_streams[name] = PointCloudStream(int(data["tcp_port"]))
            self._lidar_streams[name].submit(serialize_message(msg))
        if data.get("livox_tcp_port"):
            # Isaac 5.1 reports a truncated timestamp array length through
            # Replicator, while the native GPU buffer contains one uint64 per
            # XYZ point. Copy the native buffer using the verified XYZ count.
            import warp as wp
            from isaacsim.sensors.rtx import get_gmo_data
            node = annotator.get_node()
            ptr = node.get_attribute("outputs:timestampPtr").get()
            device_index = node.get_attribute("outputs:cudaDeviceIndex").get()
            native_timestamps = wp.array(
                ptr=ptr, dtype=wp.uint64, shape=(len(xyz),),
                device=f"cuda:{device_index}" if device_index >= 0 else "cpu", copy=False,
            ).numpy().copy()
            # Only read scalar GMO header fields; array fields can be GPU
            # pointers and must not be dereferenced as host numpy arrays.
            gmo = get_gmo_data(self._lidar_gmo[name].get_data())
            offset = stamp_ns - int(gmo.frameEnd.timestampNs)
            if name not in self._lidar_clock_offsets or abs(offset - self._lidar_clock_offsets[name]) > 100_000_000:
                # RTX's clock continues while the timeline is paused. Restart
                # partial packet assembly when the clock relation changes.
                self._lidar_clock_offsets[name] = offset
                self._livox_packetizers.pop(name, None)
            result = dict(result)
            result["timestamp"] = native_timestamps.astype(np.int64) + self._lidar_clock_offsets[name]
            if name not in self._livox_packetizers:
                from .livox_packets import LivoxPackets
                self._livox_packetizers[name] = LivoxPackets(int(data.get("livox_scan_lines", 4)))
                print(f"[Livox] Native per-point timestamps: {len(native_timestamps)}; RTX-to-sim offset_ns={self._lidar_clock_offsets[name]}", flush=True)
            packets = self._livox_packetizers[name].push(result, stamp_ns)
            key = "livox:" + name
            if key not in self._lidar_streams:
                from .pointcloud_stream import PointCloudStream
                self._lidar_streams[key] = PointCloudStream(int(data["livox_tcp_port"]), capacity=16)
            for payload in packets:
                self._lidar_streams[key].submit(payload)
        if name not in self._lidar_first_cloud_logged:
            print(f"[LiDAR] {name}: {msg.width} points, {len(msg.data)} bytes", flush=True)
            self._lidar_first_cloud_logged.add(name)
    
    def add_lidar_writter(self, data):

        # List all the available writers: print(rep.writers.WriterRegistry._writers)
        render_prod_path = rep.create.render_product(data["stage_prim_path"], [1, 1], name=data["lidar_name"])

        # Create the writer for the lidar
        writer = rep.writers.get("RtxLidarROS2PublishPointCloud")
        writer.initialize(nodeNamespace=self._namespace + str(self._id), topicName=data["lidar_name"] + "/pointcloud", frameId=data["lidar_name"])
        writer.attach([render_prod_path])

        # Add the writer to the dictionary
        self.graphical_sensors_writers[data["lidar_name"]] = [writer]

        if not data.get("publish_laserscan", True):
            return

        # Create the writer for publishing a laser scan message along with the point cloud
        writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
        writer.initialize(nodeNamespace=self._namespace + str(self._id), topicName=data["lidar_name"] + "/laserscan", frameId=data["lidar_name"])
        writer.attach([render_prod_path])
        self.graphical_sensors_writers[data["lidar_name"]].append(writer)

    def input_reference(self):
        """
        Method that is used to return the latest target angular velocities to be applied to the vehicle

        Returns:
            A list with the target angular velocities for each individual rotor of the vehicle
        """
        return self.input_ref

    def update(self, dt: float):
        """
        Method that when implemented, should be used to update the state of the backend and the information being sent/received
        from the communication interface. This method will be called by the simulation on every physics step
        """

        # In this case, do nothing as we are sending messages as soon as new data arrives from the sensors and state
        # and updating the reference for the thrusters as soon as receiving from ROS2 topics
        # Just poll for new ROS 2 messages in a non-blocking way
        rclpy.spin_once(self.node, timeout_sec=0)

    def start(self):
        """
        Method that when implemented should handle the begining of the simulation of vehicle
        """
        # Reset the reference for the thrusters
        self.input_ref = [0.0 for i in range(self._num_rotors)]

    def stop(self):
        """
        Method that when implemented should handle the stopping of the simulation of vehicle
        """
        for stream in self._lidar_streams.values():
            stream.close()
        self._lidar_streams.clear()
        self._livox_packetizers.clear()
        self._lidar_clock_offsets.clear()
        # Reset the reference for the thrusters
        self.input_ref = [0.0 for i in range(self._num_rotors)]

    def reset(self):
        """
        Method that when implemented, should handle the reset of the vehicle simulation to its original state
        """
        # Reset the reference for the thrusters
        self.input_ref = [0.0 for i in range(self._num_rotors)]
