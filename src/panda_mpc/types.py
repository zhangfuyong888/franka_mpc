"""Data crossing control-layer boundaries. All angles rad, time s, torque Nm."""
from dataclasses import dataclass, field
import numpy as np


def vector(x, n=7):
    a = np.asarray(x, dtype=float).copy()
    if a.shape != (n,) or not np.isfinite(a).all():
        raise ValueError(f'Expected finite shape ({n},), got {a}')
    return a


@dataclass
class RobotState:
    t: float
    q: np.ndarray
    dq: np.ndarray
    p_ee: np.ndarray
    v_ee: np.ndarray


@dataclass
class GoalEvent:
    goal_id: int
    scheduled_time: float
    position_world: np.ndarray

    def __post_init__(self):
        self.position_world = vector(self.position_world, 3)
        if not np.isfinite(self.scheduled_time) or self.scheduled_time < 0:
            raise ValueError('Goal time must be finite and nonnegative')


@dataclass
class GoalRecord:
    event: GoalEvent
    status: str = 'pending'
    activated_at: float | None = None
    first_command_at: float | None = None
    q_goal: np.ndarray | None = None
    reached_at: float | None = None
    superseded_at: float | None = None
    reason: str = ''
    ik: dict = field(default_factory=dict)


@dataclass
class IKResult:
    success: bool
    q_goal: np.ndarray | None
    position_error: float
    orientation_error: float
    iterations: int
    elapsed: float
    reason: str


@dataclass
class MPCResult:
    success: bool
    status: str
    q_pred: np.ndarray | None = None
    dq_pred: np.ndarray | None = None
    a_pred: np.ndarray | None = None
    primal_residual: float = float('nan')
    dual_residual: float = float('nan')
    iterations: int = 0
    elapsed: float = 0.0
    violation: float = float('nan')
    margin: float = float('nan')
    # input_rejected：尚未求解；solver_failure：求解器失败；solution_rejected：解未通过复核。
    failure_kind: str = ''
    operating_violation: float = float('nan')


@dataclass
class JointCommand:
    q_des: np.ndarray
    dq_des: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    tau_ff: np.ndarray
    stamp: float
    valid_until: float
    goal_id: int

    def __post_init__(self):
        for name in ('q_des', 'dq_des', 'kp', 'kd', 'tau_ff'):
            setattr(self, name, vector(getattr(self, name)))
        if np.any(self.kp < 0) or np.any(self.kd < 0):
            raise ValueError('Negative MIT gains')
        if not np.isfinite([self.stamp, self.valid_until]).all() or self.valid_until < self.stamp:
            raise ValueError('Invalid command lifetime')


@dataclass
class ActuationResult:
    position_feedback: np.ndarray
    velocity_feedback: np.ndarray
    feedforward: np.ndarray
    tau_raw: np.ndarray
    tau_applied: np.ndarray
    saturated: np.ndarray
    emergency: bool = False


@dataclass
class MonitorEvent:
    t: float
    kind: str
    reason: str
    goal_id: int
    values: dict = field(default_factory=dict)
