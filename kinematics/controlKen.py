import pybullet as p
import pybullet_data
import time
import math
import os


# --- 1. The Kinematics Class ---
class QuadrupedLeg:
    def __init__(self, L1, L2, L3, is_right=False):
        self.L1 = -L1 if is_right else L1
        self.L2 = L2
        self.L3 = L3

    def inverse_kinematics(self, x, y, z, knee_bends_backwards=True):
        Ld_squared = y ** 2 + z ** 2
        if Ld_squared < self.L1 ** 2:
            raise ValueError("Target is unreachable: Too close to body center.")

        Z_offset = math.sqrt(Ld_squared - self.L1 ** 2)
        gamma = math.atan2(y, -z)
        alpha = math.atan2(self.L1, Z_offset)
        theta1 = gamma - alpha

        L_squared = x ** 2 + Z_offset ** 2
        if L_squared > (self.L2 + self.L3) ** 2:
            raise ValueError("Target is unreachable: Beyond maximum leg extension.")

        cos_theta3 = (L_squared - self.L2 ** 2 - self.L3 ** 2) / (2 * self.L2 * self.L3)
        cos_theta3 = max(-1.0, min(1.0, cos_theta3))

        theta3 = math.acos(cos_theta3)
        if not knee_bends_backwards:
            theta3 = -theta3

        beta = math.atan2(x, Z_offset)
        delta = math.atan2(self.L3 * math.sin(theta3), self.L2 + self.L3 * math.cos(theta3))
        theta2 = beta - delta

        return theta1, theta2, theta3


# --- 2. Robot Geometry & Stability Tweaks ---
L1, L2, L3 = 0.05, 0.2, 0.2
body_length, body_width, body_height = 0.4, 0.2, 0.05
stance_width_offset = 0.03  # Artificially widens the stance for stability

legs = {
    "FL": QuadrupedLeg(L1, L2, L3, is_right=False),
    "FR": QuadrupedLeg(L1, L2, L3, is_right=True),
    "BL": QuadrupedLeg(L1, L2, L3, is_right=False),
    "BR": QuadrupedLeg(L1, L2, L3, is_right=True)
}
corners = [("FL", 1, 1), ("FR", 1, -1), ("BL", -1, 1), ("BR", -1, -1)]

# --- 3. Generate URDF with Collision ---
urdf = f"""<?xml version="1.0"?>
<robot name="quadruped">
  <link name="base_link">
    <visual><geometry><box size="{body_length} {body_width} {body_height}"/></geometry><material name="gray"><color rgba="0.3 0.3 0.3 1"/></material></visual>
    <collision><geometry><box size="{body_length} {body_width} {body_height}"/></geometry></collision>
    <inertial><mass value="2.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
"""
for name, dx, dy in corners:
    actual_L1 = L1 if dy == 1 else -L1
    urdf += f"""
  <link name="{name}_coxa">
    <visual><geometry><cylinder length="{abs(actual_L1)}" radius="0.02"/></geometry><origin rpy="1.5708 0 0" xyz="0 {actual_L1 / 2} 0"/><material name="red"><color rgba="0.8 0 0 1"/></material></visual>
    <collision><geometry><cylinder length="{abs(actual_L1)}" radius="0.02"/></geometry><origin rpy="1.5708 0 0" xyz="0 {actual_L1 / 2} 0"/></collision>
    <inertial><mass value="0.2"/><inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/></inertial>
  </link>
  <link name="{name}_femur">
    <visual><geometry><cylinder length="{L2}" radius="0.02"/></geometry><origin rpy="0 0 0" xyz="0 0 {-L2 / 2}"/></visual>
    <collision><geometry><cylinder length="{L2}" radius="0.02"/></geometry><origin rpy="0 0 0" xyz="0 0 {-L2 / 2}"/></collision>
    <inertial><mass value="0.2"/><inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/></inertial>
  </link>
  <link name="{name}_tibia">
    <visual><geometry><sphere radius="0.02"/></geometry><origin rpy="0 0 0" xyz="0 0 {-L3}"/><material name="blue"><color rgba="0 0 0.8 1"/></material></visual>
    <visual><geometry><cylinder length="{L3}" radius="0.015"/></geometry><origin rpy="0 0 0" xyz="0 0 {-L3 / 2}"/><material name="blue"><color rgba="0 0 0.8 1"/></material></visual>
    <collision><geometry><sphere radius="0.02"/></geometry><origin rpy="0 0 0" xyz="0 0 {-L3}"/></collision>
    <collision><geometry><cylinder length="{L3}" radius="0.015"/></geometry><origin rpy="0 0 0" xyz="0 0 {-L3 / 2}"/></collision>
    <inertial><mass value="0.2"/><inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/></inertial>
  </link>
  <joint name="{name}_shoulder_roll" type="revolute"><parent link="base_link"/><child link="{name}_coxa"/><origin xyz="{dx * body_length / 2} {dy * body_width / 2} 0" rpy="0 0 0"/><axis xyz="1 0 0"/><limit effort="30" velocity="10" lower="-1.5" upper="1.5"/></joint>
  <joint name="{name}_shoulder_pitch" type="revolute"><parent link="{name}_coxa"/><child link="{name}_femur"/><origin xyz="0 {actual_L1} 0" rpy="0 0 0"/><axis xyz="0 1 0"/><limit effort="30" velocity="10" lower="-1.5" upper="1.5"/></joint>
  <joint name="{name}_knee_pitch" type="revolute"><parent link="{name}_femur"/><child link="{name}_tibia"/><origin xyz="0 0 {-L2}" rpy="0 0 0"/><axis xyz="0 1 0"/><limit effort="30" velocity="10" lower="-3.0" upper="3.0"/></joint>
"""
urdf += "</robot>"

with open("temp_quadruped.urdf", "w") as f: f.write(urdf)

# --- 4. Initialize PyBullet ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.81)

robot_id = p.loadURDF("temp_quadruped.urdf", [0, 0, 0.35], useFixedBase=False)

num_joints = p.getNumJoints(robot_id)
joint_indices = {}
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    joint_name = info[1].decode('utf-8')
    joint_indices[joint_name] = i
    if "tibia" in info[12].decode('utf-8'):
        p.changeDynamics(robot_id, i, lateralFriction=2.0)

for _ in range(100): p.stepSimulation()

# --- 5. OMNIDIRECTIONAL CREEP GAIT LOOP ---
gait_clock = 0.0
# Cycles per second (Creep is slower and more deliberate than a Trot)
speed_multiplier = 1.25
max_stride = 0.06
step_height = 0.06
resting_z = -0.21  # Dropped the CoM significantly for balance

# Smoothing variables
current_vx, current_vy, current_omega = 0.0, 0.0, 0.0
smoothing_factor = 0.05  # Lower = smoother acceleration

# Phase offsets for a 4-beat Creep Gait (0.0 to 1.0)
phase_offsets = {"FL": 0.0, "BR": 0.25, "FR": 0.5, "BL": 0.75}
swing_fraction = 0.25  # Each foot spends 25% of the cycle in the air

print("\n" + "=" * 40)
print("STABLE CREEP GAIT ACTIVE!")
print("Click PyBullet window, then use:")
print("  [W] / [S] : Forward / Backward")
print("  [A] / [D] : Strafe Left / Right")
print("  [Q] / [E] : Turn Left / Right")
print("=" * 40 + "\n")

try:
    while p.isConnected():
        # 1. Read Inputs
        keys = p.getKeyboardEvents()
        target_vx, target_vy, target_omega = 0.0, 0.0, 0.0

        if ord('w') in keys and (keys[ord('w')] & p.KEY_IS_DOWN): target_vx = -1.0
        if ord('s') in keys and (keys[ord('s')] & p.KEY_IS_DOWN): target_vx = 1.0
        if ord('a') in keys and (keys[ord('a')] & p.KEY_IS_DOWN): target_vy = 1.0
        if ord('d') in keys and (keys[ord('d')] & p.KEY_IS_DOWN): target_vy = -1.0
        if ord('q') in keys and (keys[ord('q')] & p.KEY_IS_DOWN): target_omega = 1.0
        if ord('e') in keys and (keys[ord('e')] & p.KEY_IS_DOWN): target_omega = -1.0

        # 2. Apply Low-Pass Filter to smooth out jerks
        current_vx += (target_vx - current_vx) * smoothing_factor
        current_vy += (target_vy - current_vy) * smoothing_factor
        current_omega += (target_omega - current_omega) * smoothing_factor

        is_moving = abs(current_vx) + abs(current_vy) + abs(current_omega) > 0.05
        if is_moving:
            gait_clock += (1. / 240.) * speed_multiplier

        for name, dx, dy in corners:
            # 3. Superposition Math
            x_offset = dx * (body_length / 2)
            y_offset = dy * (body_width / 2)

            leg_vx = current_vx - (current_omega * y_offset * 5.0)
            leg_vy = current_vy + (current_omega * x_offset * 5.0)

            # 4. Normalized Phase Logic
            local_clock = (gait_clock + phase_offsets[name]) % 1.0

            if local_clock < swing_fraction and is_moving:
                # SWING PHASE (Airborne)
                progress = local_clock / swing_fraction
                # Map 0->1 progress to a sine wave for smooth Z lifting
                target_z = resting_z + step_height * math.sin(progress * math.pi)
                # Move foot from back (-1) to front (+1)
                stride_multiplier = -1.0 + 2.0 * progress
            else:
                # STANCE PHASE (On ground, pushing body)
                if is_moving:
                    progress = (local_clock - swing_fraction) / (1.0 - swing_fraction)
                else:
                    progress = 0.5  # Default to neutral center when stopped
                target_z = resting_z
                # Move foot from front (+1) to back (-1) to push the robot
                stride_multiplier = 1.0 - 2.0 * progress

            # Apply multiplier to commanded velocities
            target_x = (leg_vx * max_stride) * stride_multiplier

            base_y = (L1 + stance_width_offset) if dy == 1 else -(L1 + stance_width_offset)
            target_y = base_y + (leg_vy * max_stride) * stride_multiplier

            # 5. Apply IK
            leg = legs[name]
            try:
                knee_dir = True if dx == 1 else False
                t1, t2, t3 = leg.inverse_kinematics(target_x, target_y, target_z, knee_bends_backwards=knee_dir)

                p.setJointMotorControl2(robot_id, joint_indices[f"{name}_shoulder_roll"], p.POSITION_CONTROL,
                                        targetPosition=t1, force=30)
                p.setJointMotorControl2(robot_id, joint_indices[f"{name}_shoulder_pitch"], p.POSITION_CONTROL,
                                        targetPosition=t2, force=30)
                p.setJointMotorControl2(robot_id, joint_indices[f"{name}_knee_pitch"], p.POSITION_CONTROL,
                                        targetPosition=t3, force=30)
            except ValueError:
                pass

        p.stepSimulation()
        time.sleep(1. / 240.)

except KeyboardInterrupt:
    pass
finally:
    p.disconnect()
    if os.path.exists("temp_quadruped.urdf"):
        os.remove("temp_quadruped.urdf")