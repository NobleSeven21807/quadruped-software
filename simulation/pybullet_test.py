import pybullet as p
import pybullet_data
import time
import math

# -------------------------
# 1️⃣ Connect to PyBullet
# -------------------------
physicsClient = p.connect(p.GUI)  # Opens the simulation window
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Access default URDFs
p.setGravity(0, 0, -9.8)

# -------------------------
# 2️⃣ Load the ground
# -------------------------
planeId = p.loadURDF("plane.urdf")

# -------------------------
# 3️⃣ Load a simple 3-joint leg URDF
# For now, we'll use KUKA arm as a placeholder
# -------------------------
robotId = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

# -------------------------
# 4️⃣ Print joint info
# -------------------------
num_joints = p.getNumJoints(robotId)
print("Number of joints:", num_joints)
print("Movable joints info:")
for i in range(num_joints):
    info = p.getJointInfo(robotId, i)
    print(f"Joint {i}: name={info[1].decode('utf-8')}, type={info[2]}, lower={info[8]}, upper={info[9]}")

# -------------------------
# 5️⃣ Simulation loop
# -------------------------
# We'll control the first three joints as a single leg
joint_indices = [0, 1, 2]  # adjust if needed based on the printout
while True:
    t = time.time()
    
    # Example motion: sine wave for first three joints
    for idx in joint_indices:
        angle = 0.5 * math.sin(t + idx)  # phase shift each joint
        p.setJointMotorControl2(
            robotId,
            idx,
            p.POSITION_CONTROL,
            targetPosition=angle
        )
    
    # Step simulation
    p.stepSimulation()
    time.sleep(1/240)

    # Print end-effector (foot) position
    # Replace 6 with the correct link index for the "foot" in your model
    link_index = 6
    link_state = p.getLinkState(robotId, link_index)
    foot_pos = link_state[0]  # x, y, z