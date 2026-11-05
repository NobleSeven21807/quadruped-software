"""Gait scheduler: turns a velocity command into foot targets every tick.

A gait is a phase clock (0 -> 1 per stride) plus a per-leg offset into that
clock. While a foot is in stance it slides backward under the body, which
pushes the body along; in swing it lifts and moves to its next touchdown spot.
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class VelocityCommand:
    """Desired body motion: m/s forward, m/s left, rad/s counter-clockwise."""

    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0

    def is_zero(self):
        return self.vx == 0 and self.vy == 0 and self.yaw_rate == 0


@dataclass(frozen=True)
class GaitParams:
    period: float  # s per stride
    duty_factor: float  # fraction of the stride each foot is on the ground
    swing_height: float  # m the foot lifts during swing
    phase_offsets: dict = field(default_factory=dict)  # leg role -> offset in [0, 1)

    @property
    def stance_time(self):
        return self.period * self.duty_factor


# Duty factors leave a window where all four feet are down, so the robot can
# stop there when the command goes to zero.
TROT = GaitParams(
    period=0.8,
    duty_factor=0.6,
    swing_height=0.03,
    phase_offsets={"front_left": 0.0, "back_right": 0.0, "front_right": 0.5, "back_left": 0.5},
)

WALK = GaitParams(
    period=1.6,
    duty_factor=0.8,
    swing_height=0.03,
    phase_offsets={"front_left": 0.0, "back_right": 0.25, "front_right": 0.5, "back_left": 0.75},
)


def _smoothstep(s):
    """0 -> 1 with zero velocity at both ends, so feet lift off and land gently."""
    return 0.5 - 0.5 * np.cos(np.pi * s)


class _FootState:
    def __init__(self, home):
        self.home = np.array(home, dtype=float)
        self.position = self.home.copy()
        self.liftoff = self.home.copy()
        self.in_swing = False


class GaitScheduler:
    def __init__(self, params, home_feet, leg_roles):
        """
        params: GaitParams
        home_feet: leg name -> neutral foot position (body frame) when standing
        leg_roles: role ("front_left", ...) -> leg name
        """
        self.params = params
        self.phase = 0.0
        self.feet = {leg: _FootState(home) for leg, home in home_feet.items()}
        self.offsets = {leg: params.phase_offsets[role] for role, leg in leg_roles.items()}

    def reset(self):
        self.phase = 0.0
        for foot in self.feet.values():
            foot.position = foot.home.copy()
            foot.in_swing = False

    def is_standing(self):
        """All feet down at home, so the clock can stop without a jump."""
        return all(
            not foot.in_swing and np.allclose(foot.position, foot.home, atol=1e-3)
            for foot in self.feet.values()
        )

    def step(self, command, dt):
        """Advance the gait by dt seconds. Returns leg name -> foot target (body frame)."""
        if command.is_zero() and self.is_standing():
            return self.foot_targets()

        self.phase = (self.phase + dt / self.params.period) % 1.0
        for leg, foot in self.feet.items():
            leg_phase = (self.phase + self.offsets[leg]) % 1.0
            if leg_phase < self.params.duty_factor:
                self._update_stance(foot, command, dt)
            else:
                swing_progress = (leg_phase - self.params.duty_factor) / (1 - self.params.duty_factor)
                self._update_swing(foot, command, swing_progress)
        return self.foot_targets()

    def foot_targets(self):
        return {leg: foot.position.copy() for leg, foot in self.feet.items()}

    def _update_stance(self, foot, command, dt):
        foot.in_swing = False
        foot.position[:2] -= self._ground_velocity(foot.position, command) * dt
        foot.position[2] = foot.home[2]

    def _update_swing(self, foot, command, progress):
        if not foot.in_swing:
            foot.in_swing = True
            foot.liftoff = foot.position.copy()

        # Land ahead of home by half a stance stroke, so stance sweeps evenly
        # from front to back of home.
        touchdown = foot.home.copy()
        touchdown[:2] += 0.5 * self._ground_velocity(foot.home, command) * self.params.stance_time

        blend = _smoothstep(progress)
        foot.position[:2] = (1 - blend) * foot.liftoff[:2] + blend * touchdown[:2]
        lift = self.params.swing_height * np.sin(np.pi * progress)
        if command.is_zero() and np.allclose(foot.liftoff, foot.home, atol=1e-3):
            lift = 0.0  # already home, no need to step
        foot.position[2] = foot.home[2] + lift

    @staticmethod
    def _ground_velocity(position, command):
        """How fast the body moves over a foot planted at position (body frame xy)."""
        x, y = position[0], position[1]
        return np.array([
            command.vx - command.yaw_rate * y,
            command.vy + command.yaw_rate * x,
        ])
