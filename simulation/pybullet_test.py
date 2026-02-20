import pybullet as p
import pybullet_data
import time
import math
import os

from kinematics.forward_kinematics import two_joint_leg

# -------------------------
# 1️⃣ Connect to PyBullet GUI
# -------------------------
physicsClient = p.connect(p.GUI)  # opens the simulation window
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # for plane.urdf

# -------------------------
# 2️⃣ Load ground plane
# -------------------------
planeId = p.loadURDF("plane.urdf")

# -------------------------
# 3️⃣ Load your 2-joint leg URDF
# -------------------------
# Build absolute path to avoid path errors
script_dir = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(script_dir, "urdf", "three_joint_leg.urdf")

leg_positions = [
    (0.2, -0.1, 0.5),  # front right
    (0.2,  0.1, 0.5),  # front left
    (-0.2,-0.1, 0.5),  # rear right
    (-0.2, 0.1, 0.5),  # rear left
]

legs = []
for pos in leg_positions:
    leg_id = p.loadURDF(urdf_path, basePosition=pos, useFixedBase=True)
    legs.append(leg_id)


# -------------------------
# 5️⃣ Set which joints are movable
# -------------------------
# Adjust indices if needed based on the printout
abduction_index = 0
hip_index = 1
knee_index = 2

# Link index of the foot (lower leg)
foot_link_index = 2

def leg_ik(x, y, z, L1=0.2, L2=0.2):
    """
    Compute abduction, hip, knee angles to reach foot target (x, y, z)
    relative to leg base (hip joint).
    """
    # Abduction angle from lateral offset
    abduction = math.atan2(y, x)
    
    # Project into X'-Z plane for planar 2-joint IK
    x_prime = math.sqrt(x**2 + y**2)
    r = math.sqrt(x_prime**2 + z**2)
    
    # Knee angle via law of cosines
    cos_knee = (L1**2 + L2**2 - r**2) / (2 * L1 * L2)
    theta_knee = math.acos(max(min(cos_knee, 1), -1))
    
    # Hip angle
    gamma = math.atan2(x_prime, z)
    cos_hip = (r**2 + L1**2 - L2**2) / (2 * L1 * r)
    alpha = math.acos(max(min(cos_hip, 1), -1))
    theta_hip = gamma - alpha
    
    return abduction, theta_hip, theta_knee

# -------------------------
# 6️⃣ Simulation loop
# -------------------------
while True:
    t = time.time()

    for i, leg_id in enumerate(legs):
        phase = i % 2 * math.pi
        foot_x = 0.2 + 0.05 * math.sin(t + phase)
        foot_y = leg_positions[i][1]
        foot_z = -0.2 + 0.05 * math.sin(t + phase)

        hip_angle = 0.5 * math.sin(t + phase)
        knee_angle = 0.3 * math.cos(t + phase)

        p.setJointMotorControl2(leg_id, abduction_index, p.POSITION_CONTROL, targetPosition=abduction_index)
        p.setJointMotorControl2(leg_id, hip_index, p.POSITION_CONTROL, targetPosition=hip_angle)
        p.setJointMotorControl2(leg_id, knee_index, p.POSITION_CONTROL, targetPosition=knee_angle)

    # Step the simulation
    p.stepSimulation()
    time.sleep(1/240)  # 240 Hz simulation
