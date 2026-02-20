import math

def two_joint_leg(theta_hip, theta_knee, L1=0.2, L2=0.2):
    """
    Compute the foot position (x, z) of a 2-joint leg in the X-Z plane.
    
    theta_hip: hip joint angle in radians
    theta_knee: knee joint angle in radians
    L1: upper leg length
    L2: lower leg length
    """
    x = L1 * math.sin(theta_hip) + L2 * math.sin(theta_hip + theta_knee)
    z = L1 * math.cos(theta_hip) + L2 * math.cos(theta_hip + theta_knee)
    return (x, z)