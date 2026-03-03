import pybullet as p
import pybullet_data
import time
import math
import os


# --- 1. The Updated Kinematics Class ---
class QuadrupedLeg:
    def __init__(self, L1, L2, L3, is_right=False):
        # If it's a right leg, the physical shoulder offset points in the -Y direction
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


# --- 2. Robot Geometry Parameters ---
L1, L2, L3 = 0.05, 0.2, 0.2
body_length = 0.4  # X dimension
body_width = 0.2  # Y dimension
body_height = 0.05  # Z dimension

# Instantiate the 4 legs (Front-Left, Front-Right, Back-Left, Back-Right)
legs = {
    "FL": QuadrupedLeg(L1, L2, L3, is_right=False),
    "FR": QuadrupedLeg(L1, L2, L3, is_right=True),
    "BL": QuadrupedLeg(L1, L2, L3, is_right=False),
    "BR": QuadrupedLeg(L1, L2, L3, is_right=True)
}

# --- 3. Procedurally Generate the Quadruped URDF ---
urdf = f"""<?xml version="1.0"?>
<robot name="quadruped">
  <link name="base_link">
    <visual>
      <geometry><box size="{body_length} {body_width} {body_height}"/></geometry>
      <material name="gray"><color rgba="0.3 0.3 0.3 1"/></material>
    </visual>
  </link>
"""

# Define corners: (Name, X_multiplier, Y_multiplier)
corners = [("FL", 1, 1), ("FR", 1, -1), ("BL", -1, 1), ("BR", -1, -1)]

for name, dx, dy in corners:
    actual_L1 = L1 if dy == 1 else -L1
    urdf += f"""
  <link name="{name}_coxa">
    <visual><geometry><cylinder length="{abs(actual_L1)}" radius="0.02"/></geometry>
    <origin rpy="1.5708 0 0" xyz="0 {actual_L1 / 2} 0"/><material name="red"><color rgba="0.8 0 0 1"/></material></visual>
  </link>
  <link name="{name}_femur">
    <visual><geometry><cylinder length="{L2}" radius="0.02"/></geometry>
    <origin rpy="0 0 0" xyz="0 0 {-L2 / 2}"/></visual>
  </link>
  <link name="{name}_tibia">
    <visual><geometry><cylinder length="{L3}" radius="0.015"/></geometry>
    <origin rpy="0 0 0" xyz="0 0 {-L3 / 2}"/><material name="blue"><color rgba="0 0 0.8 1"/></material></visual>
  </link>

  <joint name="{name}_shoulder_roll" type="revolute">
    <parent link="base_link"/><child link="{name}_coxa"/>
    <origin xyz="{dx * body_length / 2} {dy * body_width / 2} 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="20" velocity="10" lower="-1.5" upper="1.5"/>
  </joint>
  <joint name="{name}_shoulder_pitch" type="revolute">
    <parent link="{name}_coxa"/><child link="{name}_femur"/>
    <origin xyz="0 {actual_L1} 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="20" velocity="10" lower="-1.5" upper="1.5"/>
  </joint>
  <joint name="{name}_knee_pitch" type="revolute">
    <parent link="{name}_femur"/><child link="{name}_tibia"/>
    <origin xyz="0 0 {-L2}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="20" velocity="10" lower="-3.0" upper="3.0"/>
  </joint>
"""
urdf += "</robot>"

with open("temp_quadruped.urdf", "w") as f:
    f.write(urdf)

# --- 4. Initialize PyBullet ---
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)  # Keep gravity off so it hangs in the air for IK testing

robot_id = p.loadURDF("temp_quadruped.urdf", [0, 0, 0.5], useFixedBase=True)

# Map joint names to indices dynamically
joint_indices = {p.getJointInfo(robot_id, i)[1].decode('utf-8'): i for i in range(p.getNumJoints(robot_id))}

print("\n--- Starting Trotting Gait Loop ---")

# --- 5. The Main Loop ---
speed = 4.0
stride_length = 0.08
step_height = 0.06
resting_z = -0.25

try:
    while p.isConnected():
        t = time.time()

        for name, dx, dy in corners:
            # 1. Phase shift for a trot: Diagonal pairs (FL & BR, FR & BL) move together
            phase_offset = 0 if name in ["FL", "BR"] else math.pi

            # 2. Calculate cyclical trajectory
            cycle = t * speed + phase_offset

            # X moves back and forth
            target_x = stride_length * math.cos(cycle)

            # Y stays straight down from the shoulder
            target_y = L1 if dy == 1 else -L1

            # Z lifts up during the forward swing, and stays flat during the backward push
            if math.sin(cycle) > 0:  # Swing phase (foot in the air)
                target_z = resting_z + step_height * math.sin(cycle)
            else:  # Stance phase (foot on the ground)
                target_z = resting_z

            # 3. Apply IK to this specific leg
            leg = legs[name]
            try:
                # Front knees bend backward, back knees bend forward (standard dog/cat setup)
                knee_dir = True if dx == 1 else False

                t1, t2, t3 = leg.inverse_kinematics(target_x, target_y, target_z, knee_bends_backwards=knee_dir)

                # 4. Command the motors
                p.setJointMotorControl2(robot_id, joint_indices[f"{name}_shoulder_roll"], p.POSITION_CONTROL,
                                        targetPosition=t1)
                p.setJointMotorControl2(robot_id, joint_indices[f"{name}_shoulder_pitch"], p.POSITION_CONTROL,
                                        targetPosition=t2)
                p.setJointMotorControl2(robot_id, joint_indices[f"{name}_knee_pitch"], p.POSITION_CONTROL,
                                        targetPosition=t3)

            except ValueError as e:
                pass  # Ignore unreachable targets during fast testing

        p.stepSimulation()
        time.sleep(1. / 240.)

except KeyboardInterrupt:
    pass
finally:
    p.disconnect()
    if os.path.exists("temp_quadruped.urdf"):
        os.remove("temp_quadruped.urdf")