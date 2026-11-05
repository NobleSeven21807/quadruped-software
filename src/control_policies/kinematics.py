"""Leg forward/inverse kinematics. Pure math: no MuJoCo or hardware imports.

Body frame: origin at the centre of the four hip-abduction axes,
x forward, y left, z up. Joint angles are in radians and use the sim's
convention (0 = the CAD pose), so the same numbers drive the MuJoCo model.
"""

from dataclasses import dataclass

import numpy as np

LEG_NAMES = ("a", "b", "c", "d")
JOINT_NAMES = ("hip_abduction", "hip_pitch", "knee")

# Matches the actuator names in quadruped_model.SERVO_JOINTS.
ACTUATOR_ORDER = tuple(f"leg_{leg}_{joint}" for leg in LEG_NAMES for joint in JOINT_NAMES)

# Joint travel either side of the CAD pose (matches quadruped_model.JOINT_RANGE_DEG).
JOINT_LIMITS = np.deg2rad([30.0, 60.0, 60.0])


# --- Measured from the current Onshape export --------------------------------
# Everything below is in the CAD/URDF root frame at the CAD pose (all joints 0),
# measured from the MuJoCo model. Re-measure after re-exporting the CAD.

# ASSUMPTION: CAD +y is the robot's front. The knees all point toward CAD -y,
# which suggests +y (knees point backward), but this is not confirmed.
# If the front is CAD -y, flip FORWARD_IS_CAD_PLUS_Y and the leg roles swap.
FORWARD_IS_CAD_PLUS_Y = True

CAD_BODY_ORIGIN = np.array([0.57125, -0.20495, 0.0631])

# Per leg: (abduction, hip pitch, knee) as (anchor point, axis), then the foot contact point.
CAD_LEGS = {
    "a": {
        "joints": [((0.5111, -0.2739, 0.0631), (0, 1, 0)),
                   ((0.4848, -0.3025, -0.0195), (1, 0, 0)),
                   ((0.5229, -0.3565, -0.1129), (1, 0, 0))],
        "foot": (0.5197, -0.3287, -0.2331),
    },
    "b": {
        "joints": [((0.5111, -0.1360, 0.0631), (0, 1, 0)),
                   ((0.4847, -0.1075, -0.0195), (1, 0, 0)),
                   ((0.4910, -0.1614, -0.1129), (1, 0, 0))],
        "foot": (0.5196, -0.1336, -0.2331),
    },
    "c": {
        "joints": [((0.6314, -0.1361, 0.0631), (0, -1, 0)),
                   ((0.6577, -0.1076, -0.0195), (1, 0, 0)),
                   ((0.6514, -0.1615, -0.1129), (1, 0, 0))],
        "foot": (0.6482, -0.1337, -0.2331),
    },
    # The URDF tree is rooted at leg d's foot, so leg d's joints are defined
    # foot-to-chassis. Positive angles rotate the chassis about the leg instead
    # of the leg about the chassis, which flips the effective axis.
    "d": {
        "joints": [((0.6314, -0.2739, 0.0631), (0, 1, 0)),
                   ((0.6578, -0.3025, -0.0195), (-1, 0, 0)),
                   ((0.6515, -0.3565, -0.1129), (-1, 0, 0))],
        "foot": (0.6483, -0.3287, -0.2331),
    },
}
# ------------------------------------------------------------------------------


def cad_to_body(vector, is_point=True):
    """Convert a CAD-frame point or direction into the body frame."""
    v = np.asarray(vector, dtype=float)
    if is_point:
        v = v - CAD_BODY_ORIGIN
    forward = 1.0 if FORWARD_IS_CAD_PLUS_Y else -1.0
    return np.array([forward * v[1], -forward * v[0], v[2]])


@dataclass(frozen=True)
class LegGeometry:
    """One leg at the CAD pose, in the body frame.

    Abduction rotates about the body x axis; hip pitch and knee rotate about
    the body y axis. Each axis sign says which way a positive angle turns.
    """

    name: str
    abduction_anchor: np.ndarray
    hip_anchor: np.ndarray
    knee_anchor: np.ndarray
    foot: np.ndarray
    abduction_sign: float  # effective axis = sign * body x
    hip_sign: float  # effective axis = sign * body y
    knee_sign: float  # effective axis = sign * body y

    @property
    def hip_position(self):
        """Where the leg attaches to the body, used for gait and turning."""
        return np.array([self.hip_anchor[0], self.foot[1], self.abduction_anchor[2]])


def _leg_from_cad(name, cad):
    anchors = [cad_to_body(anchor) for anchor, _ in cad["joints"]]
    axes = [cad_to_body(axis, is_point=False) for _, axis in cad["joints"]]
    return LegGeometry(
        name=name,
        abduction_anchor=anchors[0],
        hip_anchor=anchors[1],
        knee_anchor=anchors[2],
        foot=cad_to_body(cad["foot"]),
        abduction_sign=float(np.sign(axes[0][0])),
        hip_sign=float(np.sign(axes[1][1])),
        knee_sign=float(np.sign(axes[2][1])),
    )


LEGS = {name: _leg_from_cad(name, cad) for name, cad in CAD_LEGS.items()}


def _leg_role(leg):
    end = "front" if leg.hip_anchor[0] > 0 else "back"
    side = "left" if leg.foot[1] > 0 else "right"
    return f"{end}_{side}"


# e.g. LEG_ROLES["front_left"] == "b" (with the front assumption above)
LEG_ROLES = {_leg_role(leg): name for name, leg in LEGS.items()}


def _wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _rotate_about_x(point, pivot, angle):
    """Rotate in the y-z plane (right-hand rule about +x)."""
    y, z = point[1] - pivot[1], point[2] - pivot[2]
    c, s = np.cos(angle), np.sin(angle)
    return np.array([point[0], pivot[1] + c * y - s * z, pivot[2] + s * y + c * z])


def _rotate_about_y(point, pivot, angle):
    """Rotate in the z-x plane (right-hand rule about +y)."""
    z, x = point[2] - pivot[2], point[0] - pivot[0]
    c, s = np.cos(angle), np.sin(angle)
    return np.array([pivot[0] + s * z + c * x, point[1], pivot[2] + c * z - s * x])


def leg_fk(joint_angles, leg):
    """Foot position in the body frame for (abduction, hip pitch, knee) angles."""
    q_ab, q_hip, q_knee = joint_angles
    foot = _rotate_about_y(leg.foot, leg.knee_anchor, leg.knee_sign * q_knee)
    foot = _rotate_about_y(foot, leg.hip_anchor, leg.hip_sign * q_hip)
    return _rotate_about_x(foot, leg.abduction_anchor, leg.abduction_sign * q_ab)


def _planar_angle(vector_zx):
    """Angle of a (z, x) vector, measured the way _rotate_about_y turns it."""
    return np.arctan2(vector_zx[1], vector_zx[0])


def leg_ik(foot_target, leg):
    """Joint angles that put the foot at foot_target (body frame).

    Returns (angles, reachable). If the target is out of reach the leg points
    at it as far as it can and reachable is False.
    """
    target = np.asarray(foot_target, dtype=float) - leg.abduction_anchor
    reachable = True

    # 1. Abduction. Hip pitch and knee can't move the foot along their own axis,
    #    so the foot always sits at a fixed sideways offset in the leg's plane.
    #    Abduction swings that plane until it passes through the target.
    offset = leg.foot[1] - leg.abduction_anchor[1]
    radial_sq = target[1] ** 2 + target[2] ** 2 - offset**2
    if radial_sq < 0:
        reachable, radial_sq = False, 0.0
    leg_plane_z = -np.sqrt(radial_sq)  # foot below the abduction axis
    abduction = np.arctan2(target[2], target[1]) - np.arctan2(leg_plane_z, offset)

    # 2. Hip pitch and knee form a 2-link arm in the leg plane (z, x coordinates).
    hip = leg.hip_anchor - leg.abduction_anchor
    thigh = (leg.knee_anchor - leg.hip_anchor)[[2, 0]]
    shank = (leg.foot - leg.knee_anchor)[[2, 0]]
    to_foot = np.array([leg_plane_z - hip[2], target[0] - hip[0]])
    l1, l2, dist = np.linalg.norm(thigh), np.linalg.norm(shank), np.linalg.norm(to_foot)

    # Law of cosines gives the angle between thigh and shank. Keep the knee
    # bending the same way it does in the CAD pose.
    cad_bend = _wrap(_planar_angle(shank) - _planar_angle(thigh))
    cos_bend = (dist**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if abs(cos_bend) > 1:
        reachable, cos_bend = False, np.clip(cos_bend, -1.0, 1.0)
    bend = np.copysign(np.arccos(cos_bend), cad_bend)
    knee = bend - cad_bend

    c, s = np.cos(knee), np.sin(knee)
    hip_to_foot = thigh + np.array([c * shank[0] - s * shank[1], s * shank[0] + c * shank[1]])
    hip_pitch = _planar_angle(to_foot) - _planar_angle(hip_to_foot)

    angles = np.array([
        _wrap(abduction) * leg.abduction_sign,
        _wrap(hip_pitch) * leg.hip_sign,
        _wrap(knee) * leg.knee_sign,
    ])
    return angles, reachable


def within_limits(joint_angles):
    return bool(np.all(np.abs(joint_angles) <= JOINT_LIMITS + 1e-9))
