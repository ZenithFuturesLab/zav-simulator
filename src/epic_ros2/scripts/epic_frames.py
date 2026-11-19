"""Rigid IMU/body conversion and LIO-world -> ZAV North/East/Up command alignment."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

def imu_to_body(position, quaternion, velocity_child, angular_child, mount_position, mount_quaternion):
    imu=Rotation.from_quat(quaternion);body=imu*Rotation.from_quat(mount_quaternion).inv()
    lever=body.apply(mount_position);omega=imu.apply(angular_child)
    p=np.asarray(position)-lever
    v=imu.apply(velocity_child)-np.cross(omega,lever)
    return p,body.as_quat(),body.inv().apply(v),body.inv().apply(omega)

def command_alignment(lio_position, lio_yaw, flight_position, flight_yaw):
    angle=flight_yaw-math.pi/2+lio_yaw
    c,s=math.cos(angle),math.sin(angle)
    a=np.array([[c,-s],[s,c]])@np.array([[0.,1.],[1.,0.]])
    offset=np.asarray(flight_position,dtype=float).copy();offset[:2]-=a@np.asarray(lio_position)[:2];offset[2]-=lio_position[2]
    return a,offset,angle

def project_command(position,yaw,alignment):
    a,offset,angle=alignment;p=np.asarray(position,dtype=float).copy();p[:2]=a@p[:2];p+=offset
    heading=(angle+math.pi/2-yaw+math.pi)%(2*math.pi)-math.pi
    return p,heading

def project_trajectory(position, velocity, acceleration, yaw, yaw_rate, alignment):
    """EPIC world -> PX4 local NED, using exactly the existing position alignment.

    Derivatives use the fixed alignment's linear part only. EPIC yaw is ENU-like;
    the existing NEU mapping reverses yaw direction, hence NED yaw-rate=-yaw-rate.
    Acceleration is a kinematic derivative, with no gravity added or subtracted.
    """
    p,heading=project_command(position,yaw,alignment)
    values=[]
    for derivative in (velocity,acceleration):
        v=np.asarray(derivative,dtype=float).copy()
        v[:2]=alignment[0]@v[:2]
        values.append(v)
    p[2]*=-1
    for v in values:v[2]*=-1
    return p,values[0],values[1],heading,-float(yaw_rate)
