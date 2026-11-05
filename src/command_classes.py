from control_policies.classical_policy import ClassicalPolicy
from control_policies.gait import VelocityCommand
from control_policies.kinematics import ACTUATOR_ORDER


class QuadrupedRobot:
    """Joins a control policy to a backend, so the same code drives sim or hardware.

    backend: any backends.backend.RobotBackend (MuJoCo now, servos on the Pi later).
    """

    def __init__(self, backend, policy=None):
        self.backend = backend
        self.policy = policy or ClassicalPolicy()
        self.command = VelocityCommand()

    def stand(self):
        self.command = VelocityCommand()
        self.backend.reset(self.policy.reset())

    def move(self, command, dt):
        """Follow command (gait.VelocityCommand) for dt seconds."""
        self.command = command
        self.backend.send_joint_targets(self.policy.step(command, dt))
        self.backend.advance(dt)

    @property
    def problem_legs(self):
        """Legs whose last target was out of reach or clamped to a joint limit."""
        return self.policy.problem_legs

    def is_running(self):
        return self.backend.is_running()

    def close(self):
        self.backend.close()
