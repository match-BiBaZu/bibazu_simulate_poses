import numpy as np

class MyQuaternions:
    def __init__(self):
        pass

    # Quaternion operations that the find_poses function requires
    @staticmethod
    def quat_conjugate(quat):
        # Returns the conjugate of a quaternion
        return np.array([quat[0], -quat[1], -quat[2], -quat[3]])

    @staticmethod
    def quat_multiply(q1, q2):
        # Multiplies two quaternions with the same behaviour as MATLAB
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return np.array([w, x, y, z])

    @staticmethod
    def quat_to_rot_matrix(quat):
        # Converts a quaternion to a rotation matrix MANUALLY
        w, x, y, z = quat
        
        # Compute the rotation matrix elements
        R = np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])

        return R

    @staticmethod
    def normalize_quat(quat):
        """Normalizes a quaternion"""
        return quat / np.linalg.norm(quat)