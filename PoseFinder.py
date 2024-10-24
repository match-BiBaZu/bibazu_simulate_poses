
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from stl import Mesh  # To read STL files
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class PoseFinder:
    def __init__(self): #default values
        # This is the current name of the workpiece
        self.workpiece_name = 'Teil_1'

        # This is the total number of simulations - usually 1000
        self.simulation_number = 1000
        
        # This is the current file path of the data stored relative to the script
        self.data_path = Path(__file__).parent / 'MHI_Data'

        # This is the current file path of the workpiece stls relative to the script
        self.workpiece_path =  Path(__file__).parent / 'Workpieces'

        # These are the arrays that are used to store the stable orientations determined by the simulations
        self.array_location = np.zeros((self.simulation_number, 3))
        self.array_quaternion_blend = np.zeros((self.simulation_number, 5))
        
        # This is the loaded STL file of the workpiece
        self.workpiece_stl = Mesh.from_file(str(self.workpiece_path / (self.workpiece_name + '.STL')))
        
    # Overrides the parameters defined in init, is done this way as you can have a flexible number of arguments
    def config(self, **kwargs):

        if 'workpiece_name' in kwargs:
            self.workpiece_name = kwargs['workpiece_name']
        if 'data_path' in kwargs:
            self.data_path = kwargs['data_path']
        if 'workpiece_path' in kwargs:
            self.workpiece_path = kwargs['workpiece_path']
        if 'simulation_number' in kwargs:
            self.simulation_number = kwargs['simulation_number']
        
        # These are the arrays that are used to store the stable orientations determined by the simulations
        self.array_location = np.zeros((self.simulation_number, 3))
        self.array_quaternion_blend = np.zeros((self.simulation_number, 5))
        
        # This is the loaded STL file of the workpiece
        self.workpiece_stl = Mesh.from_file(str(self.workpiece_path / (self.workpiece_name + '.STL')))
    
    def import_orientation_csv(self):
        # Import data from the simulation CSV files into arrays
        self.array_location = np.loadtxt(str(self.data_path / (self.workpiece_name + '_simulated_data_export_matlab_location.txt')), dtype=float).reshape(-1, 3)
        self.array_quaternion_blend[:, :4] = np.loadtxt(str(self.data_path / (self.workpiece_name + '_simulated_data_export_matlab_quaternion.txt')), dtype=float).reshape(-1, 4)


    # Quaternion operations that the find_poses function requres
    @staticmethod
    def quat_conjugate(quat):
        #Returns the conjugate of a quaternion
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

    # This function classifies pose of workpieces according to their orientation determined in quaternion space, then converts to euler
    # (original function from Torge)
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
                            #print(n)
                            #print(f"same pose at j {j}")
                        #else:
                            #print(f"different pose at j {j}")

                #print(f"here n iterates: {n}")
                self.array_quaternion_blend[i, 4] = n
                n += 1

    def find_poses_quat(self):

        n = 1  # Initialize the cluster label counter
        rot_diff_threshold = 45  # Threshold that can be adjusted as needed (in degrees)

        # Loop over each quaternion
        for i in range(len(self.array_quaternion_blend)):

            if self.array_quaternion_blend[i, 4] == 0.0:  # Check the 5th column (index 4) for assignment
                self.array_quaternion_blend[i, 4] = n  # Assign the current quaternion to the current cluster

                quat_i = self.array_quaternion_blend[i, :4]  # Extract the quaternion (first 4 elements)

                for j in range(i+1, len(self.array_quaternion_blend)):
                    if self.array_quaternion_blend[j, 4] == 0:  # Check if this quaternion has not yet been assigned a cluster
                        quat_j = self.array_quaternion_blend[j, :4]  # Extract the quaternion

                        # Normalize the quaternions
                        quat_i = self.normalize_quat(quat_i)
                        quat_j = self.normalize_quat(quat_j)

                        # Calculate the distance between the two quaternions
                        quat_diff = np.abs(self.quat_multiply(quat_i,self.quat_conjugate(quat_j)))
                        print(quat_diff)

                        # The scalar component of the quaternion is closer to 1 if the rotation is small and vice versa
                        # Therefore if the rotation is smaller than the threshold the scalar compoent will be bigger than
                        # the cosine of the threshold
                        if self.array_quaternion_blend[j, 4] == 0.0:  # If pose not classified
                            if quat_diff[0] >= np.cos((rot_diff_threshold*np.pi)/360): # Check the scalar component
                                self.array_quaternion_blend[j, 4] = n  # Assign the same cluster label
                                print(n)
                                print(f"same pose at j {j}")
                            else:
                                print(f"different pose at j {j}")

                self.array_quaternion_blend[i, 4] = n
                n += 1  # Move to the next cluster label


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
    
    def plot_poses_quat(self):
        
            # Load STL file
            points_stl = self.workpiece_stl.vectors.reshape(-1, 3)  # STL points
            cList_stl = np.arange(len(points_stl)).reshape(-1, 3)  # Connectivity list for trimesh

            # The overall number of poses
            total_poses = int(np.max(self.array_quaternion_blend[:, 4]))

            # Figure 1: meshes of frequency of stable poses
            fig = plt.figure(figsize=(12, 8), constrained_layout=True)

            # Step 1: Precompute all transformed points once to avoid recalculating
            m_values = self.array_quaternion_blend[:, 4]  # Pose numbers
            unique_poses = np.unique(m_values)  # Get the unique poses
            num_poses = len(unique_poses)  # Number of unique poses

            rotated_points_by_pose = []
            for m in unique_poses:
                # Get quaternion for the current pose
                pose_indices = np.where(self.array_quaternion_blend[:, 4] == m)[0]
                quat = self.array_quaternion_blend[pose_indices[0], :4]

                # Compute rotation matrix once for the pose
                rotm = self.quat_to_rot_matrix(quat)

                # Rotate points using the rotation matrix (efficient matrix multiplication)
                pointsR = np.dot(points_stl, rotm.T)  # Equivalent to points * rotm in MATLAB

                # Flip the points around the Z-axis
                pointsR[:, 2] = -pointsR[:, 2]
                
                # Store the rotated points for later use
                rotated_points_by_pose.append(pointsR)

            # Step 2: Compute global axis limits across all poses
            all_rotated_points = np.vstack(rotated_points_by_pose)  # Stack all rotated points into one array
            x_min, x_max = np.min(all_rotated_points[:, 0]), np.max(all_rotated_points[:, 0])
            y_min, y_max = np.min(all_rotated_points[:, 1]), np.max(all_rotated_points[:, 1])
            z_min, z_max = np.min(all_rotated_points[:, 2]), np.max(all_rotated_points[:, 2])

            # Calculate grid dimensions dynamically based on the number of poses
            cols = 3  # Set a reasonable number of columns to make each plot larger
            rows = int(np.ceil(num_poses / cols))

            # Adjust figure size based on the number of subplots (increase for more subplots)
            fig.set_size_inches(cols * 4, rows * 4)

            # Step 3: Plot each pose using global limits, using precomputed rotated points
            for a, pointsR in enumerate(rotated_points_by_pose, start=1):
                ax = fig.add_subplot(rows, cols, a, projection='3d')
                
                # Plot STL mesh using precomputed points and stored rotated points
                ax.add_collection3d(Poly3DCollection(pointsR[cList_stl], edgecolors= 'k', facecolors=[0.6, 0.6, 0.6]))

                # Set consistent global axis limits
                ax.set_xlim([x_min, x_max])
                ax.set_ylim([y_min, y_max])
                ax.set_zlim([z_min, z_max])
                ax.set_title(f'Pose {unique_poses[a-1]}')  # Use the actual pose number
                #ax.axis('off')

                # Optionally: Set axis labels for clarity
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')

            # Use tight_layout to remove unnecessary space between subplots
            plt.tight_layout()
            plt.show()

            # Figure 2: histogram of frequency of stable poses
            stable_poses_frequency, bin_edges = np.histogram(self.array_quaternion_blend[:, 4], bins=np.arange(1, total_poses + 2))
            
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

            for m in range(1, total_poses + 1):
                abs_str = str(stable_poses_frequency[m - 1])
                percentage_str = str(round(stable_poses_frequency[m - 1] / (len(self.array_quaternion_blend) / 100), 2))
                labels.append(f"Pose {m} (n = {abs_str})")
                labels_pie.append(f"Pose {m} ({percentage_str}%)")

            max_pose_idx = np.argmax(stable_poses_frequency)
            explode = np.zeros(total_poses)
            explode[max_pose_idx] = 0.1  # Adjust the explode value to make the gap smaller

            # Plot the pie chart
            plt.pie(stable_poses_frequency, explode=explode, labels=labels_pie)
            plt.title(f"Natural Resting Position -" + self.workpiece_name)
            plt.legend(labels, loc='center left', bbox_to_anchor=(1.2, 0.5))  # Add legend to the right side
            plt.show()
