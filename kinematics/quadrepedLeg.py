import math


class QuadrupedLeg:
    def __init__(self, L1, L2, L3):
        """
        L1: Length of the coxa (shoulder offset along Y)
        L2: Length of the femur (upper leg)
        L3: Length of the tibia (lower leg)
        """
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

    def forward_kinematics(self, theta1, theta2, theta3):
        """
        Calculates the (x, y, z) foot position given joint angles.
        Angles should be in radians.
        """
        # Calculate the depth of the leg in its local 2D plane
        Z_offset = self.L2 * math.cos(theta2) + self.L3 * math.cos(theta2 + theta3)

        # Calculate X (Forward axis)
        x = self.L2 * math.sin(theta2) + self.L3 * math.sin(theta2 + theta3)

        # Calculate Y and Z by rotating the local plane around the X axis by theta1
        y = self.L1 * math.cos(theta1) + Z_offset * math.sin(theta1)
        z = self.L1 * math.sin(theta1) - Z_offset * math.cos(theta1)

        return x, y, z

    def inverse_kinematics(self, x, y, z, knee_bends_backwards=True):
        """
        Calculates joint angles (theta1, theta2, theta3) to reach target (x, y, z).
        Returns angles in radians.
        """
        # --- 1. Solve for theta1 and local Z_offset in the Y-Z plane ---
        Ld_squared = y ** 2 + z ** 2

        # Check if target is physically too close to the body center (inside the shoulder width)
        if Ld_squared < self.L1 ** 2:
            raise ValueError("Target is unreachable: Too close to body center in Y-Z plane.")

        Z_offset = math.sqrt(Ld_squared - self.L1 ** 2)

        # Angle to target in Y-Z plane minus the inner triangle angle
        gamma = math.atan2(y, -z)
        alpha = math.atan2(self.L1, Z_offset)
        theta1 = gamma - alpha

        # --- 2. Solve for theta2 and theta3 in the X-Z_offset plane ---
        L_squared = x ** 2 + Z_offset ** 2

        # Check if the target is further than the leg can stretch
        if L_squared > (self.L2 + self.L3) ** 2:
            raise ValueError("Target is unreachable: Beyond maximum leg extension.")

        # Law of Cosines to find knee angle (theta3)
        cos_theta3 = (L_squared - self.L2 ** 2 - self.L3 ** 2) / (2 * self.L2 * self.L3)

        # Prevent math domain errors from floating point precision issues near 1.0 or -1.0
        cos_theta3 = max(-1.0, min(1.0, cos_theta3))

        theta3 = math.acos(cos_theta3)
        if not knee_bends_backwards:
            theta3 = -theta3  # Bends forward like a bird/digitigrade

        # Calculate femur angle (theta2)
        beta = math.atan2(x, Z_offset)
        delta = math.atan2(self.L3 * math.sin(theta3), self.L2 + self.L3 * math.cos(theta3))
        theta2 = beta - delta

        return theta1, theta2, theta3