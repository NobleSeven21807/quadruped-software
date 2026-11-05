"""Classical walking policy: velocity command -> gait -> IK -> 12 joint angles.

The output is in kinematics.ACTUATOR_ORDER, in radians, sim convention.
Any backend (MuJoCo now, servos later) takes these angles unchanged.
"""

import numpy as np

from .gait import TROT, GaitScheduler
from .kinematics import JOINT_LIMITS, LEG_NAMES, LEG_ROLES, LEGS, leg_ik

# m from the hip-abduction axes down to the feet (CAD pose is 0.296). The knee
# has only 60 deg of travel; at 0.27 a full stride still leaves ~8 deg spare.
DEFAULT_STAND_HEIGHT = 0.27


def home_foot_position(leg, stand_height):
    """Foot directly below the hip-pitch axis, in line with its CAD sideways position."""
    return np.array([leg.hip_anchor[0], leg.foot[1], -stand_height])


class ClassicalPolicy:
    def __init__(self, gait=TROT, stand_height=DEFAULT_STAND_HEIGHT):
        home_feet = {name: home_foot_position(LEGS[name], stand_height) for name in LEG_NAMES}
        self.scheduler = GaitScheduler(gait, home_feet, LEG_ROLES)
        self.problem_legs = []  # legs whose last target was unreachable or hit a joint limit

    def reset(self):
        """Back to standing. Returns the standing joint angles."""
        self.scheduler.reset()
        return self.joint_targets(self.scheduler.foot_targets())

    def step(self, command, dt):
        """Advance dt seconds following command (gait.VelocityCommand). Returns 12 angles."""
        feet = self.scheduler.step(command, dt)
        return self.joint_targets(feet)

    def joint_targets(self, feet):
        """IK for every leg, clamped to the joint limits."""
        self.problem_legs = []
        angles = []
        for name in LEG_NAMES:
            q, reachable = leg_ik(feet[name], LEGS[name])
            clamped = np.clip(q, -JOINT_LIMITS, JOINT_LIMITS)
            if not reachable or not np.allclose(q, clamped):
                self.problem_legs.append(name)
            angles.append(clamped)
        return np.concatenate(angles)
