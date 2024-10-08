
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from stl import Mesh  # To read STL files
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class PoseFinder:
    def __init__(self):
        # This is the current name of the workpiece
        self.workpiece_name = 'Teil_1'
        
        # This is the current file path of the data stored relative to the script
        self.data_path = Path(__file__).parent / 'MHI_Data'

        # This is the current file path of the workpiece stls relative to the script
        self.workpiece_path =  Path(__file__).parent / 'Workpieces'

        # This is the loaded STL file of the workpiece
        self.workpiece_stl = Mesh.from_file(str(self.workpiece_path / (self.workpiece_name + '.stl')))
        
        # These are the arrays that are used to store the stable orientations determined by the simulations
        self.array_location = np.zeros((1000, 3))
        self.array_rotation_blend = np.zeros((1000, 4))
        self.array_quaternion_blend = np.zeros((1000, 5))
    
    # Overrides the parameters defined in init, is done this way as you can have a flexible number of arguments
    def config(self, **kwargs):

        if 'workpiece_name' in kwargs:
            self.workpiece_name = kwargs['workpiece_name']
        if 'data_path' in kwargs:
            self.data_path = kwargs['data_path']
        if 'workpiece_path' in kwargs:
            self.workpiece_path = kwargs['workpiece_path']
    
    def import_orientation(self):

        # Import data from the simulation CSV files into arrays
        self.array_location = np.loadtxt(str(self.data_path / (self.workpiece_name + '_simulated_data_export_matlab_location.txt')), dtype=float).reshape(-1, 3)
        self.array_rotation_blend[:, :3] = np.loadtxt(str(self.data_path / (self.workpiece_name + '_simulated_data_export_matlab_rotation.txt')), dtype=float).reshape(-1, 3)*np.pi/180
        self.array_quaternion_blend[:, :4] = np.loadtxt(str(self.data_path / (self.workpiece_name + '_simulated_data_export_matlab_quaternion.txt')), dtype=float).reshape(-1, 4)


    # Quaternion operations that the find_poses function requres
    @staticmethod
    def quat_conjugate(quat):
        """Returns the conjugate of a quaternion"""
        return np.array([quat[0], -quat[1], -quat[2], -quat[3]])

    @staticmethod
    def quat_multiply(q1, q2):
        #Multiplies two quaternions with the same behaviour as MATLAB
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return np.array([w, x, y, z])

    @staticmethod
    def quat_to_rot_matrix(quat):
        #Converts a quaternion to a rotation matrix MANUALLY
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

    # This function classifies pose of workpieces according to their orientation determined in quaternion space
    def find_poses(self):

        n = 1
        eps = 0.3 #Arbitrarily determined thresholding parameter to seperate regions of rotation into poses

        for i in range(len(self.array_quaternion_blend)):
            if self.array_quaternion_blend[i, 4] == 0:
                self.array_quaternion_blend[i, 4] = n
                
                x = self.array_quaternion_blend[i, :4]  # KS1 - Quaternion (w, x, y, z)
                
                for j in range(i+1, len(self.array_quaternion_blend)):
                    y = self.array_quaternion_blend[j, :4]  # KS2 - Quaternion (w, x, y, z)
                    
                    # Normalize the quaternions
                    x = self.normalize_quat(x)
                    y = self.normalize_quat(y)
                    
                    # z quaternion = quat_conjugate(x) * y
                    z = self.quat_multiply(self.quat_conjugate(x), y)
                    
                    # Avoid division by zero if acos(z[0]) is 0
                    acos_z0 = np.arccos(z[0])
                    if np.sin(acos_z0) != 0:
                        r_x_KS1 = z[1] / np.sin(acos_z0)
                        r_y_KS1 = z[2] / np.sin(acos_z0)
                        r_z_KS1 = z[3] / np.sin(acos_z0)
                    else:
                        r_x_KS1, r_y_KS1, r_z_KS1 = 0, 0, 0

                    vector_r_1 = np.array([r_x_KS1, r_y_KS1, r_z_KS1])
                    
                    # Get rotation matrices from quaternions
                    rotm_x = self.quat_to_rot_matrix(x)
                    rotm_y = self.quat_to_rot_matrix(y)
                    rotm_z = self.quat_to_rot_matrix(z)
                    
                    # Rotate vector_r_1 using the rotation matrix
                    vector_r_0 = rotm_x @ vector_r_1
                    
                    r_x_KS0 = vector_r_0[0]
                    r_y_KS0 = vector_r_0[1]
                    r_z_KS0 = vector_r_0[2]

                    # Check if the quaternion belongs to the same pose
                    if self.array_quaternion_blend[j, 4] == 0.0:
                        if ((0.0-eps <= r_x_KS0 <= 0.0+eps) and 
                            (0.0-eps <= r_y_KS0 <= 0.0+eps)):
                            self.array_quaternion_blend[j, 4] = n
                            self.array_rotation_blend[j, 3] = n
                
                n += 1
    
    # helper function to convert euler to rotational matricies
    @staticmethod
    def euler_to_rot_matrix(ax, ay, az):
        """Converts Euler angles (ax, ay, az) to a rotation matrix"""
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(ax), -np.sin(ax)],
                       [0, np.sin(ax), np.cos(ax)]])
        
        Ry = np.array([[np.cos(ay), 0, np.sin(ay)],
                       [0, 1, 0],
                       [-np.sin(ay), 0, np.cos(ay)]])
        
        Rz = np.array([[np.cos(az), -np.sin(az), 0],
                       [np.sin(az), np.cos(az), 0],
                       [0, 0, 1]])
        
        # Combined rotation matrix
        return np.dot(np.dot(Rz, Ry), Rx)

    def plot_poses(self):

        # Load STL file
        points_stl =  self.workpiece_stl.vectors.reshape(-1, 3)  # STL points
        cList_stl = np.arange(len(points_stl)).reshape(-1, 3)  # Connectivity list for trimesh

        # The overall number of poses
        total_poses =  int(np.max(self.array_quaternion_blend[:, 4]))
        # Figure 1: meshes of frequency of stable poses
        fig = plt.figure(figsize=(12, 8))
        
        m = 1
        a = 1
    
        num_poses = len(np.unique(self.array_rotation_blend[:, 3]))  # Assuming 4th column is the pose number
        for i in range(len(self.array_rotation_blend)):
            if self.array_rotation_blend[i, 3] == m:
                
                ax_angle = self.array_rotation_blend[i, 0]
                ay_angle = self.array_rotation_blend[i, 1]
                az_angle = self.array_rotation_blend[i, 2]
                
                # Get rotation matrix from Euler angles
                rotm = self.euler_to_rot_matrix(ax_angle, ay_angle, az_angle)
                
                # Rotate points using the rotation matrix
                pointsR = np.dot(points_stl, rotm.T)  # Equivalent to points * rotm in MATLAB
                
                # **Flip the Z-axis** to correct the upside-down issue
                #pointsR[:, 2] = -pointsR[:, 2]  # Flip the Z-axis
                
                # Create 3D plot
                ax = fig.add_subplot(num_poses // 2 + num_poses % 2, 4, a, projection='3d')
                ax.add_collection3d(Poly3DCollection(pointsR[cList_stl], edgecolors='k', facecolors=[0.6, 0.6, 0.6]))
                
                # Set axis and plot limits
                ax.set_xlim([np.min(pointsR[:, 0]), np.max(pointsR[:, 0])])
                ax.set_ylim([np.min(pointsR[:, 1]), np.max(pointsR[:, 1])])
                ax.set_zlim([np.min(pointsR[:, 2]), np.max(pointsR[:, 2])])
                ax.set_title(f'Pose {m}')
                ax.axis('off')
                
                # Adjust for next tile (next plot in the grid)
                m += 1
                if a % 2 == 1:
                    a += 1
                else:
                    a += 3
        
        plt.show()

        # Figure 2: histogram of frequency of stable poses
        stable_poses_frequency, bin_edges = np.histogram(self.array_rotation_blend[:, 3], bins=np.arange(1, total_poses + 1))
        
        # Plot the histogram
        plt.figure()
        plt.bar(bin_edges[:-1], stable_poses_frequency, width=1)
        plt.xlabel('Pose')
        plt.ylabel('Frequency')
        plt.title('Pose Frequency Histogram')
        plt.show()
        
        # Figure 3: pie chart of stable poses
        plt.figure(figsize=(10, 6))
        labels = []
        labels_pie = []

        for m in range(1, total_poses):
            abs_str = str(stable_poses_frequency[m - 1])
            percentage_str = str(round(stable_poses_frequency[m - 1] / (len(self.array_quaternion_blend) / 100), 2))
            labels.append(f"Pose {m} (n = {abs_str})")
            labels_pie.append(f"Pose {m} ({percentage_str}%)")

        max_pose_idx = np.argmax(stable_poses_frequency)
        explode = np.zeros(total_poses - 1)
        explode[max_pose_idx] = 0.1  # Adjust the explode value to make the gap smaller

        # Plot the pie chart
        plt.pie(stable_poses_frequency, explode=explode, labels=labels_pie)
        plt.title(f"Natural Resting Position -" + self.workpiece_name)
        plt.legend(labels, loc='center left', bbox_to_anchor=(1, 0.5))  # Add legend to the right side
        plt.show()
