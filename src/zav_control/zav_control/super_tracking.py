"""SITL-only ENU tracking error to PX4 NED position/velocity conversion."""
import math


def tracking_setpoint(target, measured, local_ned, velocity, max_error=0.5, max_speed=1.5):
    if not math.isfinite(max_speed) or not 0.25 <= max_speed <= 3.5:
        raise ValueError('invalid SITL speed limit')
    values = (*target, *measured, *local_ned, *velocity)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite tracking data')
    error = [a-b for a, b in zip(target, measured)]
    if math.sqrt(sum(e*e for e in error)) > max_error:
        raise ValueError('tracking error exceeds limit')
    if math.sqrt(sum(v*v for v in velocity)) > max_speed:
        raise ValueError('trajectory velocity exceeds SITL limit')
    # Use simultaneous LIO/PX4 feedback to account for estimator origin and
    # IMU lever-arm offsets. Both desired and measured positions refer to LIO IMU.
    return ([local_ned[0]+error[1], local_ned[1]+error[0], local_ned[2]-error[2]],
            [velocity[1], velocity[0], -velocity[2]])


def predict_position(position, world_velocity, age):
    """Bring delayed LIO position to the reference epoch without changing stamps."""
    if not all(math.isfinite(x) for x in (*position, *world_velocity, age)):
        raise ValueError('nonfinite prediction data')
    if not -0.05 <= age <= 0.3:
        raise ValueError('prediction epoch outside tracking timestamp limits')
    return [p+v*max(0.,age) for p,v in zip(position,world_velocity)]


def feedback_at_epoch(samples, epoch):
    """Pick measured feedback at/before a command, avoiding future-IMU samples.

    IMU callbacks can run ahead of a lower-rate /clock or command callback. Keep
    real measurement stamps; do not relabel a future state or widen time guards.
    samples contains (measurement_time, value) pairs in arrival order.
    """
    if not math.isfinite(epoch):
        raise ValueError('nonfinite command epoch')
    candidates = [(stamp, value) for stamp, value in samples
                  if math.isfinite(stamp) and stamp <= epoch]
    if not candidates:
        raise ValueError('no tracking feedback at command epoch')
    return max(candidates, key=lambda sample: sample[0])[1]


def velocity_heading(velocity_enu, previous_ned_yaw, dt, rate_limit=1.0):
    """Face horizontal flight direction, with a bounded shortest-path yaw turn."""
    if not all(math.isfinite(v) for v in (*velocity_enu,previous_ned_yaw,dt,rate_limit)) or dt<0 or rate_limit<=0:
        raise ValueError('invalid velocity heading input')
    if math.hypot(velocity_enu[0],velocity_enu[1])<.2:
        return previous_ned_yaw
    desired=math.atan2(velocity_enu[0],velocity_enu[1])
    error=(desired-previous_ned_yaw+math.pi)%(2*math.pi)-math.pi
    step=max(-rate_limit*dt,min(rate_limit*dt,error))
    return (previous_ned_yaw+step+math.pi)%(2*math.pi)-math.pi


def acceleration_feedforward(acceleration_enu, max_norm=3.0):
    """Convert a bounded translational acceleration (gravity excluded) to NED."""
    if not all(math.isfinite(v) for v in acceleration_enu):
        raise ValueError('nonfinite trajectory acceleration')
    if math.sqrt(sum(v*v for v in acceleration_enu))>max_norm:
        raise ValueError('trajectory acceleration exceeds SITL limit')
    return [acceleration_enu[1],acceleration_enu[0],-acceleration_enu[2]]
