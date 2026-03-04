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


# --- 2. Robot Geometry ---
L1, L2, L3 = 0.05, 0.2, 0.2
body_length, body_width, body_height = 0.4, 0.2, 0.05

legs = {
    "FL": QuadrupedLeg(L1, L2, L3, is_right=False),
    "FR": QuadrupedLeg(L1, L2, L3, is_right=True),
    "BL": QuadrupedLeg(L1, L2, L3, is_right=False),
    "BR": QuadrupedLeg(L1, L2, L3, is_right=True)
}
corners = [("FL", 1, 1), ("FR", 1, -1), ("BL", -1, 1), ("BR", -1, -1)]

# --- 3. Generate URDF (Now with Collision Tags!) ---
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

# --- 4. Initialize PyBullet (Physics ON) ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 1. Load the ground plane
plane_id = p.loadURDF("plane.urdf")

# 2. Turn on real-world gravity
p.setGravity(0, 0, -9.81)

# 3. Unbolt the robot (useFixedBase=False) and drop it from Z=0.4m
robot_id = p.loadURDF("temp_quadruped.urdf", [0, 0, 0.4], useFixedBase=False)

# 4. Add friction to the feet!
num_joints = p.getNumJoints(robot_id)
joint_indices = {}
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    joint_name = info[1].decode('utf-8')
    joint_indices[joint_name] = i

    # If this link is a tibia (which holds the foot), crank up the friction
    if "tibia" in info[12].decode('utf-8'):
        p.changeDynamics(robot_id, i, lateralFriction=1.5, spinningFriction=0.1, rollingFriction=0.1)

# Let the robot drop and settle into a standing position for a second
for _ in range(240):
    p.stepSimulation()
    time.sleep(1. / 240.)

# --- 5. Interactive Main Loop ---
speed = 8.0  # Sped up slightly so momentum helps keep it from falling over
stride_length = 0.06
step_height = 0.05
resting_z = -0.25
current_demo = 1

print("\n" + "=" * 40)
print("PHYSICS DEMO STARTED! GRAVITY ON!")
print(" [1] Standard Trot")
print(" [2] Crab Walk")
print(" [3] Wide-Stance Trot")
print(" [4] Swimming Motion")
print("=" * 40 + "\n")

try:
    while p.isConnected():
        t = time.time()
        keys = p.getKeyboardEvents()

        if ord('1') in keys and (keys[ord('1')] & p.KEY_WAS_TRIGGERED):
            current_demo = 1
        elif ord('2') in keys and (keys[ord('2')] & p.KEY_WAS_TRIGGERED):
            current_demo = 2
        elif ord('3') in keys and (keys[ord('3')] & p.KEY_WAS_TRIGGERED):
            current_demo = 3
        elif ord('4') in keys and (keys[ord('4')] & p.KEY_WAS_TRIGGERED):
            current_demo = 4

        for name, dx, dy in corners:
            phase_offset = 0 if name in ["FL", "BR"] else math.pi
            cycle = t * speed + phase_offset

            base_y = L1 if dy == 1 else -L1
            swing_phase = math.sin(cycle) > 0
            target_z = resting_z + step_height * math.sin(cycle) if swing_phase else resting_z

            if current_demo == 1:
                target_x = stride_length * math.cos(cycle)
                target_y = base_y
            elif current_demo == 2:
                target_x = 0
                target_y = base_y + stride_length * math.cos(cycle)
            elif current_demo == 3:
                target_x = stride_length * math.cos(cycle)
                splay_offset = 0.06 if dy == 1 else -0.06
                target_y = base_y + splay_offset
            elif current_demo == 4:
                target_x = stride_length * math.cos(cycle)
                splay_amount = (stride_length * 0.8) * math.sin(cycle)
                target_y = base_y + splay_amount if dy == 1 else base_y - splay_amount

            leg = legs[name]
            try:
                knee_dir = True if dx == 1 else False
                t1, t2, t3 = leg.inverse_kinematics(target_x, target_y, target_z, knee_bends_backwards=knee_dir)

                # In a physics simulation, we increase the max force (effort) so the legs can lift the body mass
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