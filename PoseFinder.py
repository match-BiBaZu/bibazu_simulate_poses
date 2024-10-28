
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from stl import Mesh  # To read STL files
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from enum import Enum

class PoseFindingMode(Enum):
    TORGE = 'torge'
    QUAT = 'quaternion'
    QUAT_COMPARE = 'quat_compare'


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
        self.array_quaternion_blend = np.zeros((self.simulation_number, 6))
        self.array_pre_impulse_quaternion_blend = np.zeros((self.simulation_number, 6))

        # These are the arrays that are used to store the angular velocities determined by the simulation
        self.array_angular_velocity = np.zeros((self.simulation_number, 3))
        self.array_pre_impulse_angular_velocity = np.zeros((self.simulation_number, 3))

        # These are the arrays that are used to store the number of contact points determined by the simulation
        self.array_contact_points = np.zeros((self.simulation_number, 1))
        self.array_pre_impulse_contact_points = np.zeros((self.simulation_number, 1))    

        # This is the loaded STL file of the workpiece
        self.workpiece_stl = Mesh.from_file(str(self.workpiece_path / (self.workpiece_name + '.STL')))
    
        # Initialize counters for each category
        self.successful_reoriented_stable = 0
        self.successful_reoriented_unstable = 0
        self.unsuccessful_reoriented_stable = 0
        self.unsuccessful_reoriented_unstable = 0

        
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
        if 'mode' in kwargs:
            if isinstance(kwargs['mode'], PoseFindingMode):
                self.mode = kwargs['mode']
            else:
                raise ValueError("Invalid mode. Must be of type PoseFindingMode.")
        
        # These are the arrays that are used to store the stable orientations determined by the simulations
        self.array_quaternion_blend = np.zeros((self.simulation_number, 6))
        self.array_pre_impulse_quaternion_blend = np.zeros((self.simulation_number, 6))

        # These are the arrays that are used to store the angular velocities determined by the simulation
        self.array_angular_velocity = np.zeros((self.simulation_number, 3))
        self.array_pre_impulse_angular_velocity = np.zeros((self.simulation_number, 3))

        # These are the arrays that are used to store the number of contact points determined by the simulation
        self.array_contact_points = np.zeros((self.simulation_number, 1))
        self.array_pre_impulse_contact_points = np.zeros((self.simulation_number, 1))    
        # This is the loaded STL file of the workpiece
        self.workpiece_stl = Mesh.from_file(str(self.workpiece_path / (self.workpiece_name + '.STL')))
    
    def import_orientation_csv(self):
        # Import data from the simulation CSV files into arrays
        data = np.loadtxt(str(self.data_path / (self.workpiece_name + '_simulated_data_export_quaternion.txt')), dtype=float)
        
        if data.shape[1] == 4:
            self.array_quaternion_blend[:, :4] = data.reshape(-1, 4)
        elif data.shape[1] == 8:
            self.array_quaternion_blend[:, :4] = data[:, :4].reshape(-1, 4)
            self.array_pre_impulse_quaternion_blend[:, :4] = data[:, 4:].reshape(-1, 4)
        else:
            raise ValueError("Unexpected number of columns in the input file.")

        # Import angular velocity data
        angular_velocity_data = np.loadtxt(str(self.data_path / (self.workpiece_name + '_simulated_data_export_angular_velocity.txt')), dtype=float)
        
        if angular_velocity_data.shape[1] == 6:
            self.array_angular_velocity = angular_velocity_data[:, :3]
            self.array_pre_impulse_angular_velocity = angular_velocity_data[:, 3:]
        else:
            raise ValueError("Unexpected number of columns in the angular velocity input file.")

        # Import contact points data
        contact_points_data = np.loadtxt(str(self.data_path / (self.workpiece_name + '_simulated_data_export_contact_points.txt')), dtype=float)

        if contact_points_data.shape[1] == 2:
            self.array_contact_points = contact_points_data[:, 0].reshape(-1, 1)
            self.array_pre_impulse_contact_points = contact_points_data[:, 1].reshape(-1, 1)
        else:
            raise ValueError("Unexpected number of columns in the contact points input file.")

    def find_poses(self):

        if self.mode == PoseFindingMode.TORGE:
            # This one is the conversion of the original matlab code written by Torge
            # only good for workpieces landing on a plane
            self.array_quaternion_blend = self.find_poses_quat(self.array_quaternion_blend)

            self.plot_mesh_visualization(self.array_quaternion_blend)
            self.plot_frequency_histogram(self.array_quaternion_blend)
            self.plot_pose_pie_chart(self.array_quaternion_blend)
             
        elif self.mode == PoseFindingMode.QUAT:
            self.array_quaternion_blend = self.find_poses_quat(self.array_quaternion_blend)

            self.plot_mesh_visualization(self.array_quaternion_blend)
            self.plot_frequency_histogram(self.array_quaternion_blend)
            self.plot_pose_pie_chart(self.array_quaternion_blend)

        elif self.mode == PoseFindingMode.QUAT_COMPARE:
            concatenated_quaternions = np.vstack((self.array_quaternion_blend, self.array_pre_impulse_quaternion_blend))
            concatenated_quaternions = self.find_poses_quat(concatenated_quaternions)
            concatenated_quaternions = self.check_if_pose_is_stable(concatenated_quaternions, 
                                         np.vstack((self.array_contact_points,self.array_pre_impulse_contact_points)), 
                                         np.vstack((self.array_angular_velocity,self.array_pre_impulse_angular_velocity)))

            self.array_quaternion_blend = concatenated_quaternions[:self.simulation_number, :6]
            self.array_pre_impulse_quaternion_blend = concatenated_quaternions[self.simulation_number:, :6]

            self.plot_mesh_visualization(concatenated_quaternions)
            self.plot_frequency_histogram(self.array_quaternion_blend)
            self.plot_frequency_histogram(self.array_pre_impulse_quaternion_blend)

            self.find_reorientation_rate()
            self.plot_reorientation_pie_chart()
                 
        else:
            raise ValueError("Invalid mode specified.")
        
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
    def find_poses_torge(self, array_quaternion_pose):

        n = 1
        eps = 0.3  # Arbitrarily determined thresholding parameter to separate regions of rotation into poses

        for i in range(len(array_quaternion_pose)):
            if array_quaternion_pose[i, 4] == 0:
                array_quaternion_pose[i, 4] = n

                x = array_quaternion_pose[i, :4]  # KS1 - Quaternion (w, x, y, z)
                
                for j in range(i+1, len(array_quaternion_pose)):
                    y = array_quaternion_pose[j, :4]  # KS2 - Quaternion (w, x, y, z)
                    
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
                    if array_quaternion_pose[j, 4] == 0.0:
                        if ((0.0-eps <= r_x_KS0 <= 0.0+eps) and 
                            (0.0-eps <= r_y_KS0 <= 0.0+eps)):
                            array_quaternion_pose[j, 4] = n
                            #print(n)
                            #print(f"same pose at j {j}")
                        #else:
                            #print(f"different pose at j {j}")

                #print(f"here n iterates: {n}")
                array_quaternion_pose[i, 4] = n
                n += 1
        return array_quaternion_pose

    def find_poses_quat(self,array_quaternion_pose):

        n = 1  # Initialize the cluster label counter
        rot_diff_threshold = 45  # Threshold that can be adjusted as needed (in degrees)

        # Loop over each quaternion
        for i in range(len(array_quaternion_pose)):

            if array_quaternion_pose[i, 4] == 0.0:  # Check the 5th column (index 4) for assignment
                array_quaternion_pose[i, 4] = n  # Assign the current quaternion to the current cluster

                quat_i = array_quaternion_pose[i, :4]  # Extract the quaternion (first 4 elements)

                for j in range(i+1, len(array_quaternion_pose)):
                    if array_quaternion_pose[j, 4] == 0:  # Check if this quaternion has not yet been assigned a cluster
                        quat_j = array_quaternion_pose[j, :4]  # Extract the quaternion

                        # Normalize the quaternions
                        quat_i = self.normalize_quat(quat_i)
                        quat_j = self.normalize_quat(quat_j)

                        # Calculate the distance between the two quaternions
                        quat_diff = np.abs(self.quat_multiply(quat_i,self.quat_conjugate(quat_j)))
                        print(quat_diff)

                        # The scalar component of the quaternion is closer to 1 if the rotation is small and vice versa
                        # Therefore if the rotation is smaller than the threshold the scalar compoent will be bigger than
                        # the cosine of the threshold
                        if array_quaternion_pose[j, 4] == 0.0:  # If pose not classified
                            if quat_diff[0] >= np.cos((rot_diff_threshold*np.pi)/360): # Check the scalar component
                                array_quaternion_pose[j, 4] = n  # Assign the same cluster label
                                print(n)
                                print(f"same pose at j {j}")
                            else:
                                print(f"different pose at j {j}")

                array_quaternion_pose[i, 4] = n
                n += 1  # Move to the next cluster label
        return array_quaternion_pose
    

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
    
    @staticmethod
    def check_if_pose_is_stable(array_quaternion_pose, contact_points, angular_velocity):

        # Check if the pose is stable based on the angular velocity and contact points
        for i in range(len(array_quaternion_pose)):
            if ((np.linalg.norm(angular_velocity[i]) < 5.0) and (contact_points[i] > 1)):
                array_quaternion_pose[i, 5] = 1  # Set the pose to unstable
            else:
                array_quaternion_pose[i, 5] = 0  # Set the pose to stable
        return array_quaternion_pose
    
    def plot_poses_quat(self,array_quaternion_pose):
        
            # Load STL file
            points_stl = self.workpiece_stl.vectors.reshape(-1, 3)  # STL points
            cList_stl = np.arange(len(points_stl)).reshape(-1, 3)  # Connectivity list for trimesh

            # The overall number of poses
            total_poses = int(np.max(array_quaternion_pose[:, 4]))

            # Figure 1: meshes of frequency of stable poses
            fig = plt.figure(figsize=(12, 8), constrained_layout=True)

            # Step 1: Precompute all transformed points once to avoid recalculating
            m_values = array_quaternion_pose[:, 4]  # Pose numbers
            unique_poses = np.unique(m_values)  # Get the unique poses
            num_poses = len(unique_poses)  # Number of unique poses

            rotated_points_by_pose = []
            unstable_poses = []

            for m in unique_poses:
                # Get quaternion for the current pose
                pose_indices = np.where(array_quaternion_pose[:, 4] == m)[0]
                quat = array_quaternion_pose[pose_indices[0], :4]
                unstable_pose = array_quaternion_pose[pose_indices[0], 5]
                # Compute rotation matrix once for the pose
                rotm = self.quat_to_rot_matrix(quat)

                # Rotate points using the rotation matrix (efficient matrix multiplication)
                pointsR = np.dot(points_stl, rotm.T)  # Equivalent to points * rotm in MATLAB

                # Flip the points around the Z-axis
                pointsR[:, 2] = -pointsR[:, 2]
                
                # Store the rotated points for later use
                rotated_points_by_pose.append(pointsR)
                unstable_poses.append(unstable_pose)

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
                if unstable_poses[a-1] == 1:
                    ax.add_collection3d(Poly3DCollection(pointsR[cList_stl], edgecolors= 'r', facecolors=[0.6, 0.6, 0.6]))
                else:
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
            stable_poses_frequency, bin_edges = np.histogram(array_quaternion_pose[:, 4], bins=np.arange(1, total_poses + 2))
            
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
                percentage_str = str(round(stable_poses_frequency[m - 1] / (len(array_quaternion_pose) / 100), 2))
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
    
    def plot_mesh_visualization(self, array_quaternion_pose):

            # Load STL file
            points_stl = self.workpiece_stl.vectors.reshape(-1, 3)  # STL points
            cList_stl = np.arange(len(points_stl)).reshape(-1, 3)  # Connectivity list for trimesh

            # Figure 1: meshes of frequency of stable poses
            fig = plt.figure(figsize=(12, 8), constrained_layout=True)

            # Step 1: Precompute all transformed points once to avoid recalculating
            m_values = array_quaternion_pose[:, 4]  # Pose numbers
            unique_poses = np.unique(m_values)  # Get the unique poses
            num_poses = len(unique_poses)  # Number of unique poses

            rotated_points_by_pose = []
            unstable_poses = []
            
            for m in unique_poses:
                # Get quaternion for the current pose
                pose_indices = np.where(array_quaternion_pose[:, 4] == m)[0]
                quat = array_quaternion_pose[pose_indices[0], :4]
                unstable_pose = array_quaternion_pose[pose_indices[0], 5]
                # Compute rotation matrix once for the pose
                rotm = self.quat_to_rot_matrix(quat)

                # Rotate points using the rotation matrix (efficient matrix multiplication)
                pointsR = np.dot(points_stl, rotm.T)  # Equivalent to points * rotm in MATLAB

                # Flip the points around the Z-axis
                pointsR[:, 2] = -pointsR[:, 2]
                
                # Store the rotated points for later use
                rotated_points_by_pose.append(pointsR)
                unstable_poses.append(unstable_pose)
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
                if unstable_poses[a-1] == 1:
                    ax.add_collection3d(Poly3DCollection(pointsR[cList_stl], edgecolors= 'r', facecolors=[0.6, 0.6, 0.6]))
                else:
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

    @staticmethod
    def plot_frequency_histogram(array_quaternion_pose):
        total_poses = int(np.max(array_quaternion_pose[:, 4]))
        stable_poses_frequency, bin_edges = np.histogram(array_quaternion_pose[:, 4], bins=np.arange(1, total_poses + 2))
        bar_colors = []  # An empty list

        # Determine the color of each bar based on the stability of the pose
        if np.all(array_quaternion_pose[:, 5] == 1):
            bar_colors = ['red'] * total_poses
        elif np.all(array_quaternion_pose[:, 5] == 0):
            bar_colors = ['blue'] * total_poses
        else:
            bar_colors = ['red' if np.any(array_quaternion_pose[np.where(array_quaternion_pose[:, 4] == i), 5] == 1) 
                                else 'blue' for i in range(1, total_poses + 1)]

        plt.figure()
        plt.bar(bin_edges[:-1], stable_poses_frequency, width=1, color=bar_colors)
        plt.xlabel('Pose')
        plt.ylabel('Frequency')
        plt.title('Pose Frequency Histogram')
        plt.show()

    def plot_pose_pie_chart(self,array_quaternion_pose):

        total_poses = int(np.max(array_quaternion_pose[:, 4]))
        stable_poses_frequency, bin_edges = np.histogram(array_quaternion_pose[:, 4], bins=np.arange(1, total_poses + 2))

        plt.figure(figsize=(10, 6))
        labels = []
        labels_pie = []
        for m in range(1, total_poses + 1):
            abs_str = str(stable_poses_frequency[m - 1])
            percentage_str = str(round(stable_poses_frequency[m - 1] / (len(array_quaternion_pose) / 100), 2))
            labels.append(f"Pose {m} (n = {abs_str})")
            labels_pie.append(f"Pose {m} ({percentage_str}%)")

        max_pose_idx = np.argmax(stable_poses_frequency)
        explode = np.zeros(total_poses)
        explode[max_pose_idx] = 0.1  # Highlight the most frequent pose

        plt.pie(stable_poses_frequency, explode=explode, labels=labels_pie)
        plt.title(f"Natural Resting Position - " + self.workpiece_name)
        plt.legend(labels, loc='center left', bbox_to_anchor=(1.2, 0.5))
        plt.show()
    
    def find_reorientation_rate(self):
        # Iterate through the arrays to classify each row
        for i in range(self.simulation_number):
            pose_blend = self.array_quaternion_blend[i, 4]
            pose_pre_impulse = self.array_pre_impulse_quaternion_blend[i, 4]
            stable_blend = self.array_quaternion_blend[i, 5]
            stable_pre_impulse = self.array_pre_impulse_quaternion_blend[i, 5]

            if pose_blend != pose_pre_impulse:
                if stable_blend == 0 and stable_pre_impulse == 0:
                    self.successful_reoriented_stable += 1
                else:
                    self.successful_reoriented_unstable += 1
            else:
                if stable_blend == 0 and stable_pre_impulse == 0:
                    self.unsuccessful_reoriented_stable += 1
                else:
                    self.unsuccessful_reoriented_unstable += 1
    
    def plot_reorientation_pie_chart(self):
        self.find_reorientation_rate()

        # Data for the pie chart
        sizes = [self.successful_reoriented_stable, self.unsuccessful_reoriented_stable, 
             self.successful_reoriented_unstable, self.unsuccessful_reoriented_unstable]
        labels = ['Successfully Re-oriented (Stable)', 'Unsuccessfully Re-oriented (Stable)', 
              'Successfully Re-oriented (Unstable)', 'Unsuccessfully Re-oriented (Unstable)']
        colors = ['#99ff99', '#ff6666', '#ff3333', '#ff0000']  # Green for success red for failure
        explode = [0.1 if size > 0 else 0 for size in sizes]  # Explode the first slice if it has a value

        # Filter out categories with 0 results using list comprehension
        filtered_data = [(size, label, color, exp) for size, label, color, exp in zip(sizes, labels, colors, explode) if size > 0]
        
        # Unzip the filtered data
        sizes, labels, colors, explode = zip(*filtered_data) if filtered_data else ([], [], [], [])

        # Plot the pie chart
        plt.figure(figsize=(10, 6))
        plt.pie(sizes, explode=explode, labels=labels, colors=colors, autopct='%1.1f%%', startangle=140)
        plt.title('Determining Re-orientation Rate')
        plt.legend(labels, loc='center left', bbox_to_anchor=(1.2, 0.5))
        plt.show()

    def return_reorientation_rate(self):

        return np.array([self.successful_reoriented_stable, 
                         self.unsuccessful_reoriented_stable, 
                         self.successful_reoriented_unstable, 
                         self.unsuccessful_reoriented_unstable]) / self.simulation_number
