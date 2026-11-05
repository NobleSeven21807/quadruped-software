"""RobotBackend running on the MuJoCo model from quadruped_model.py.

Needs src/ on sys.path (run from src/, like the rest of the code).
"""

import time

import mujoco
import mujoco.viewer
import numpy as np

from .backend import RobotBackend, RobotState
from quadruped_model import GROUND_CLEARANCE, build_model, lowest_point

# --- Tied to the current Onshape export; update after re-exporting the CAD ----
CHASSIS_BODY = "part_1"
# Must match kinematics.FORWARD_IS_CAD_PLUS_Y.
FORWARD_IS_CAD_PLUS_Y = True
# ------------------------------------------------------------------------------


def _body_axes_in_cad():
    """Columns are the body x (forward), y (left), z (up) axes in CAD coordinates."""
    forward = 1.0 if FORWARD_IS_CAD_PLUS_Y else -1.0
    return np.array([
        [0.0, -forward, 0.0],
        [forward, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])


class MujocoBackend(RobotBackend):
    def __init__(self, joint_order, viewer=False):
        """
        joint_order: actuator names in the policy's order (kinematics.ACTUATOR_ORDER)
        viewer: open the MuJoCo viewer and run at real-time speed
        """
        self.model = build_model()
        self.data = mujoco.MjData(self.model)
        self.chassis = self.model.body(CHASSIS_BODY).id

        actuators = [self.model.actuator(name) for name in joint_order]
        joints = [self.model.actuator_trnid[a.id][0] for a in actuators]
        self.actuator_ids = np.array([a.id for a in actuators])
        self.qpos_ids = self.model.jnt_qposadr[joints]
        self.qvel_ids = self.model.jnt_dofadr[joints]
        self.free_joint = self.model.joint("base")

        # Body frame relative to the chassis, taken at the CAD pose where the
        # chassis is level.
        mujoco.mj_kinematics(self.model, self.data)
        chassis_in_cad = self.data.xmat[self.chassis].reshape(3, 3)
        self.body_in_chassis = chassis_in_cad.T @ _body_axes_in_cad()

        self.viewer = mujoco.viewer.launch_passive(self.model, self.data) if viewer else None
        self._wall_start = None

    def reset(self, joint_positions):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_ids] = joint_positions
        self._level_chassis_on_ground()
        # Start the servo targets at the pose too, so nothing twitches.
        self.data.ctrl[self.actuator_ids] = joint_positions
        mujoco.mj_forward(self.model, self.data)
        self._wall_start = None
        self._sync_viewer()

    def _level_chassis_on_ground(self):
        """The model's free joint sits on leg d's foot (the URDF root), so setting
        leg d's joints tilts the chassis. Re-place the root so the chassis is level
        and centred, then drop it onto the floor. Remove once the CAD is re-rooted.
        """
        model, data = self.model, self.data
        mujoco.mj_kinematics(model, data)
        root = self.free_joint.bodyid[0]
        root_pos, root_rot = data.xpos[root].copy(), data.xmat[root].reshape(3, 3)
        chassis_pos, chassis_rot = data.xpos[self.chassis], data.xmat[self.chassis].reshape(3, 3)

        # Level = body axes line up with world axes (world x is the robot's forward).
        level_rot = self.body_in_chassis.T
        root_in_chassis = chassis_rot.T @ (root_pos - chassis_pos)
        new_root_rot = level_rot @ chassis_rot.T @ root_rot

        qpos = self.free_joint.qposadr[0]
        data.qpos[qpos : qpos + 3] = level_rot @ root_in_chassis
        mujoco.mju_mat2Quat(data.qpos[qpos + 3 : qpos + 7], new_root_rot.flatten())
        mujoco.mj_kinematics(model, data)
        data.qpos[qpos + 2] += GROUND_CLEARANCE - lowest_point(model, data)

    def read_state(self):
        model, data = self.model, self.data
        body_rot = data.xmat[self.chassis].reshape(3, 3) @ self.body_in_chassis

        velocity = np.zeros(6)  # angular, linear (world-aligned)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, self.chassis, velocity, 0)
        mujoco.mj_rnePostConstraint(model, data)
        acceleration = np.zeros(6)  # includes gravity's reaction, like a real accelerometer
        mujoco.mj_objectAcceleration(model, data, mujoco.mjtObj.mjOBJ_BODY, self.chassis, acceleration, 0)

        orientation = np.zeros(4)
        mujoco.mju_mat2Quat(orientation, body_rot.flatten())
        return RobotState(
            time=data.time,
            joint_positions=data.qpos[self.qpos_ids].copy(),
            joint_velocities=data.qvel[self.qvel_ids].copy(),
            orientation=orientation,
            gyro=body_rot.T @ velocity[:3],
            accelerometer=body_rot.T @ acceleration[3:],
        )

    def send_joint_targets(self, joint_positions):
        self.data.ctrl[self.actuator_ids] = joint_positions

    def advance(self, dt):
        steps = max(1, round(dt / self.model.opt.timestep))
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
        self._sync_viewer()

    def _sync_viewer(self):
        if self.viewer is None:
            return
        self.viewer.sync()
        # Hold the sim to real time so it looks like the real robot.
        if self._wall_start is None:
            self._wall_start = time.perf_counter() - self.data.time
        ahead = self.data.time - (time.perf_counter() - self._wall_start)
        if ahead > 0:
            time.sleep(ahead)

    def is_running(self):
        return self.viewer is None or self.viewer.is_running()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
