"""RobotBackend for the real robot: joint angles -> servo pulses -> ESP32.

    policy angles (rad) -> ServoBackend -> USB serial -> ESP32 -> PCA9685 -> servos

Talks to firmware/esp32_servo_bridge, which expects one line of 12 pulse widths
(microseconds) in kinematics.ACTUATOR_ORDER, e.g. "1500 1500 ... 1500\n".
Pulse i drives PCA9685 channel i.

Needs src/ on sys.path (run from src/, like the rest of the code) and pyserial.
"""

import time
from dataclasses import dataclass

import numpy as np

from .backend import RobotBackend, RobotState
from control_policies.kinematics import JOINT_LIMITS

# --- Servo hardware ------------------------------------------------------------
SERVO_RANGE_DEG = 270.0  # travel over the full 500-2500 us range (180 for the 180° version)
MIN_PULSE_US = 500
MAX_PULSE_US = 2500
US_PER_DEG = (MAX_PULSE_US - MIN_PULSE_US) / SERVO_RANGE_DEG

# --- Serial link to the ESP32 ----------------------------------------------------
DEFAULT_PORT = "/dev/ttyUSB0"  # check with: ls /dev/ttyUSB* /dev/ttyACM*
DEFAULT_BAUD = 115200          # must match Serial.begin() in the firmware
READY_TIMEOUT_S = 3.0


@dataclass(frozen=True)
class JointCalibration:
    """How one joint's angle maps onto its servo.

    center_us:  pulse that puts the joint at angle 0 (the CAD pose)
    direction:  +1 or -1, flip if the joint moves the wrong way
    gear_ratio: servo degrees per joint degree (2.0 for a 16T -> 32T belt)
    """

    center_us: float = 1500.0
    direction: int = 1
    gear_ratio: float = 1.0


# Placeholder values until each servo is calibrated. Keys are actuator names
# from kinematics.ACTUATOR_ORDER; any joint not listed uses the defaults.
CALIBRATION = {
    # "leg_a_knee": JointCalibration(center_us=1520, direction=-1, gear_ratio=2.0),
}


class ServoBackend(RobotBackend):
    def __init__(self, joint_order, port=DEFAULT_PORT, baud=DEFAULT_BAUD, calibration=None):
        """
        joint_order: actuator names in the policy's order (kinematics.ACTUATOR_ORDER)
        port, baud:  the ESP32's USB serial port
        calibration: {actuator name: JointCalibration}, defaults to CALIBRATION
        """
        if len(joint_order) != 12:
            raise ValueError(f"expected 12 joints, got {len(joint_order)}")
        calibration = CALIBRATION if calibration is None else calibration
        cals = [calibration.get(name, JointCalibration()) for name in joint_order]

        self.center_us = np.array([c.center_us for c in cals])
        self.us_per_rad = np.array([c.direction * c.gear_ratio for c in cals]) * np.rad2deg(US_PER_DEG)
        # JOINT_LIMITS is (abduction, hip pitch, knee); ACTUATOR_ORDER repeats that per leg.
        self.joint_limits = np.tile(JOINT_LIMITS, 4)

        self.serial = self._open_serial(port, baud)
        self.targets = np.zeros(12)
        self._start = time.perf_counter()
        self._next_tick = None

    @staticmethod
    def _open_serial(port, baud):
        import serial  # pyserial; imported here so the sim never needs it

        link = serial.Serial(port, baud, timeout=0.5)
        # Opening the port restarts most ESP32 boards; wait for the firmware's "READY".
        deadline = time.perf_counter() + READY_TIMEOUT_S
        while time.perf_counter() < deadline:
            if link.readline().decode(errors="replace").strip() == "READY":
                return link
        link.close()
        raise TimeoutError(f"no READY from the ESP32 on {port}; is the firmware uploaded?")

    def angles_to_pulses(self, joint_positions):
        """Radians (CAD-pose convention) -> whole microseconds, clamped to the servo range."""
        angles = np.clip(joint_positions, -self.joint_limits, self.joint_limits)
        pulses = self.center_us + self.us_per_rad * angles
        return np.clip(np.rint(pulses), MIN_PULSE_US, MAX_PULSE_US).astype(int)

    def reset(self, joint_positions):
        # PWM servos can't report where they are, so this moves them at full
        # speed. Hold the robot near the standing pose (or on a stand) first.
        self.send_joint_targets(joint_positions)
        self._next_tick = None

    def read_state(self):
        # No sensors yet: report the last commanded angles. Orientation, gyro and
        # accelerometer are placeholders (level and still) until the IMU is added.
        return RobotState(
            time=time.perf_counter() - self._start,
            joint_positions=self.targets.copy(),
            joint_velocities=np.zeros(12),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            gyro=np.zeros(3),
            accelerometer=np.array([0.0, 0.0, 9.81]),
        )

    def send_joint_targets(self, joint_positions):
        self.targets = np.asarray(joint_positions, dtype=float).copy()
        pulses = self.angles_to_pulses(self.targets)
        self.serial.write((" ".join(map(str, pulses)) + "\n").encode())

    def advance(self, dt):
        """Wait until dt seconds after the previous tick, keeping a steady control rate."""
        now = time.perf_counter()
        if self._next_tick is None:
            self._next_tick = now
        self._next_tick += dt
        if self._next_tick > now:
            time.sleep(self._next_tick - now)
        else:
            self._next_tick = now  # fell behind; don't try to catch up in a burst

    def close(self):
        # The firmware keeps sending the last pulses, so the servos hold their pose.
        if self.serial.is_open:
            self.serial.close()
