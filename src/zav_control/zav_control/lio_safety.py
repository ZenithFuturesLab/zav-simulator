"""Motion-continuity-only LIO jump detection, using measurement-time intervals.

No data freshness, arrival-rate, correction, or EKF fusion failure triggers.
An armed position/attitude jump latches until a ground-side process restart.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SafetyConfig:
    position_slack: float = 0.5
    velocity_slack: float = 0.3
    max_acceleration: float = 20.0
    angle_slack: float = math.radians(20)
    max_angular_rate: float = 4.0
    prediction_horizon: float = 0.25

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')


def unit_quaternion(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('invalid quaternion')
    norm = math.hypot(*q)
    if not 0.5 <= norm <= 1.5:
        raise ValueError('invalid quaternion norm')
    return tuple(v / norm for v in q)


def rotate(q, v):
    """Rotate child-frame linear velocity into odometry world coordinates (wxyz)."""
    w, x, y, z = q
    tx, ty, tz = (2*(y*v[2]-z*v[1]), 2*(z*v[0]-x*v[2]), 2*(x*v[1]-y*v[0]))
    return (v[0]+w*tx+y*tz-z*ty, v[1]+w*ty+z*tx-x*tz,
            v[2]+w*tz+x*ty-y*tx)


class LioSafety:
    def __init__(self, config=None):
        self.cfg = config or SafetyConfig()
        self.armed = False
        self.latched = False
        self.latch_reasons = []
        self.last_odom = None
        self.residual_error = None
        self.metrics = {}
        self.comparison = 'waiting_for_odometry'

    def set_armed(self, armed, now=None):
        if self.armed and not armed and not self.latched:
            self.last_odom = None
            self.comparison = 'waiting_for_odometry'
        self.armed = armed

    def odometry(self, stamp, position, quaternion, child_velocity, now=None):
        if self.latched:
            return False
        self.residual_error = None
        self.metrics = {}
        try:
            q = unit_quaternion(quaternion)
            if (len(position) != 3 or len(child_velocity) != 3
                    or not all(math.isfinite(v) for v in (stamp, *position, *child_velocity))):
                raise ValueError('invalid odometry')
            velocity = rotate(q, child_velocity)
            if not all(math.isfinite(v) for v in velocity):
                raise ValueError('invalid velocity')
        except (ValueError, TypeError, OverflowError):
            # Invalid values are dropped, not classified as a physical jump.
            self.comparison = 'invalid_sample_dropped'
            return False
        candidate = (stamp, tuple(position), q, velocity)
        if self.last_odom is None:
            self.last_odom = candidate
            self.comparison = 'baseline_initialized'
            return True
        previous_stamp, previous_position, previous_q, previous_velocity = self.last_odom
        dt = stamp - previous_stamp
        if dt <= 0:
            self.comparison = 'nonmonotonic_sample_dropped'
            return False
        if dt > self.cfg.prediction_horizon:
            # A gap alone is not a motion jump. Start a new comparison baseline.
            self.last_odom = candidate
            self.comparison = 'gap_rebaseline'
            self.metrics['measurement_dt_s'] = dt
            return True
        predicted = [p+v*dt for p, v in zip(previous_position, previous_velocity)]
        residual = math.dist(position, predicted)
        position_limit = (self.cfg.position_slack + self.cfg.velocity_slack*dt
                          + 0.5*self.cfg.max_acceleration*dt*dt)
        cosine = min(1.0, abs(sum(a*b for a, b in zip(previous_q, q))))
        angle = 2*math.acos(cosine)
        angle_limit = self.cfg.angle_slack + self.cfg.max_angular_rate*dt
        self.metrics = dict(measurement_dt_s=dt, position_residual_m=residual,
                            position_limit_m=position_limit,
                            angle_residual_deg=math.degrees(angle),
                            angle_limit_deg=math.degrees(angle_limit))
        reasons = []
        if residual > position_limit:
            reasons.append('position_residual')
        if angle > angle_limit:
            reasons.append('attitude_residual')
        self.comparison = 'jump_detected' if reasons else 'continuous'
        self.residual_error = ','.join(reasons) if reasons else None
        if reasons and self.armed:
            self.latched = True
            self.latch_reasons = reasons
            return False
        # Ground-side mapper initialization/resets may establish a fresh baseline.
        self.last_odom = candidate
        return not reasons

    def evaluate(self, now=None):
        ready = (self.last_odom is not None and not self.latched
                 and self.comparison not in ('invalid_sample_dropped', 'jump_detected'))
        return dict(state='FAILED' if self.latched else ('READY' if ready else 'WARMUP'),
                    ready=ready, latched=self.latched, armed=self.armed,
                    reasons=list(self.latch_reasons), comparison=self.comparison,
                    metrics=dict(self.metrics))
