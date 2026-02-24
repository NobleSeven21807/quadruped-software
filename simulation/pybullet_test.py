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
    (0.1, -0.2, 0.5),  # front right
    (0.1,  0.2, 0.5),  # front left
    (-0.1,-0.2, 0.5),  # rear right
    (-0.1, 0.2, 0.5),  # rear left
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
    z_prime = z
    
    r = math.sqrt(x_prime**2 + z_prime**2)

    r = min(r, L1 + L2 - 1e-6)
    r = max(r, abs(L1 - L2) + 1e-6)

    cos_knee = (L1**2 + L2**2 - r**2) / (2 * L1 * L2)
    knee_angle = math.acos(max(min(cos_knee, 1), -1))
    
    # Hip angle
    gamma = math.atan2(-z_prime, x_prime)
    cos_alpha = (r**2 + L1**2 - L2**2) / (2 * L1 * r)
    alpha = math.acos(max(min(cos_alpha, 1), -1))
    hip_angle = gamma - alpha
    
    return abduction, hip_angle, knee_angle

# -------------------------
# 6️⃣ Simulation loop
# -------------------------
phases = [0, math.pi, 0, math.pi]
stride_length = 0.1
lift_height = 0.05
foot_x_base = 0.15
foot_z_base = -0.25

while True:
    t = time.time()

    for i, leg_id in enumerate(legs):
        phase = phases[i]
        foot_x = foot_x_base + stride_length * math.sin(t + phase)
        foot_y = 0.0
        foot_z = foot_z_base + lift_height * math.cos(t + phase)

        abduction, hip_angle, knee_angle = leg_ik(foot_x, foot_y, foot_z)

        p.setJointMotorControl2(leg_id, abduction_index, p.POSITION_CONTROL, targetPosition=abduction_index)
        p.setJointMotorControl2(leg_id, hip_index, p.POSITION_CONTROL, targetPosition=hip_angle)
        p.setJointMotorControl2(leg_id, knee_index, p.POSITION_CONTROL, targetPosition=knee_angle)

    # Step the simulation
    p.stepSimulation()
    print(f"Abduction: {abduction:.2f}, Hip: {hip_angle:.2f}, Knee: {knee_angle:.2f}")
    foot_pos = p.getLinkState(leg_id, knee_index)[0]  # position of foot link
    print("Foot position:", foot_pos)
    time.sleep(1/240)  # 240 Hz simulation
