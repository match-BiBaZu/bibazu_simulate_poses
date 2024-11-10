from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from stl import Mesh  # To read STL files
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from enum import Enum
import csv
import tf_transformations 

class PoseFinder:
    def __init__(self): #default values
        # This is the current name of the workpiece
        self.workpiece_name = 'Teil_1'

        # This is the total number of simulations - usually 1000
        self.simulation_number = 1001
        
        # This is the current file path of the input data stored relative to the script
        self.data_path = Path(__file__).parent / 'Simulation_Data' / 'Bullet_Raw_Data' / 'Logged_Simulations'

        # This is the current file path of the workpiece stls relative to the script
        self.workpiece_path =  Path(__file__).parent / 'Workpieces'

        # These are the arrays that are used to store the stable orientations determined by the simulations
        self.array_quaternion_blend = np.zeros((self.simulation_number, 5))
        self.array_pre_impulse_quaternion_blend = np.zeros((self.simulation_number, 5))

        # These are the arrays that are used to store the angular velocities determined by the simulation
        self.array_angular_velocity = np.zeros((self.simulation_number, 3))
        self.array_pre_impulse_angular_velocity = np.zeros((self.simulation_number, 3))

        # These are the arrays that are used to store the number of contact points determined by the simulation
        self.array_contact_points = np.zeros((self.simulation_number, 1))

        # THese are the arrays that are used to store the location of the workpiece
        self.array_location = np.zeros((self.simulation_number, 3))
        self.array_pre_impulse_location = np.zeros((self.simulation_number, 3))

        # This is the loaded STL file of the workpiece at scale 10
        self.workpiece_stl = Mesh.from_file(str(self.workpiece_path / (self.workpiece_name + '.STL')), scale=10)
        #self.workpiece_stl = Mesh.from_file(str(self.workpiece_path / (self.workpiece_name + '.STL')))
    
        # Initialize the simulation outcomes array to store what is considered as success as well as the different failure modes
        self.simulation_outcomes = np.zeros(self.simulation_number)
        self.sliding_distance_array = np.zeros(self.simulation_number)

        
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
            if kwargs['mode'] <= 3:
                self.mode = kwargs['mode']
            else:
                raise ValueError("Invalid mode. Mode must be 0, 1, 2 or 3.")
        
        # These are the arrays that are used to store the stable orientations determined by the simulations
        self.array_quaternion_blend = np.zeros((self.simulation_number, 5))
        self.array_pre_impulse_quaternion_blend = np.zeros((self.simulation_number, 5))

        # These are the arrays that are used to store the angular velocities determined by the simulation
        self.array_angular_velocity = np.zeros((self.simulation_number, 3))
        self.array_pre_impulse_angular_velocity = np.zeros((self.simulation_number, 3))

        # These are the arrays that are used to store the number of contact points determined by the simulation
        self.array_contact_points = np.zeros((self.simulation_number, 1))
        self.array_pre_impulse_contact_points = np.zeros((self.simulation_number, 1)) 

        # THese are the arrays that are used to store the location of the workpiece
        self.array_location = np.zeros((self.simulation_number, 3))
        self.array_pre_impulse_location = np.zeros((self.simulation_number, 3))

        # This is the loaded STL file of the workpiece
        self.workpiece_stl = Mesh.from_file(str(self.workpiece_path / (self.workpiece_name + '.STL')), scale=10)

        # Initialize the simulation outcomes array to store what is considered as success as well as the different failure modes
        self.simulation_outcomes = np.zeros(self.simulation_number)
        self.sliding_distance_array = np.zeros(self.simulation_number)
    
    def set_orientation_array(self,quaternion_array):
        self.array_quaternion_blend[:, :4] = quaternion_array[:, :4].reshape(-1, 4)
        self.array_pre_impulse_quaternion_blend[:, :4] = quaternion_array[:, 4:].reshape(-1, 4)
    
    def set_angular_velocity_array(self,angular_velocity_array):
        self.array_angular_velocity = angular_velocity_array[:, :3].reshape(-1, 3)
        self.array_pre_impulse_angular_velocity = angular_velocity_array[:, 3:].reshape(-1, 3)

    def set_location_array(self,location_array):
        self.array_location = location_array[:, :3].reshape(-1, 3)
        self.array_pre_impulse_location = location_array[:, 3:].reshape(-1, 3)

    def import_temp_csv(self):
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
            self.array_angular_velocity = angular_velocity_data[:, :3].reshape(-1, 3)
            self.array_pre_impulse_angular_velocity = angular_velocity_data[:, 3:].reshape(-1, 3)
        else:
            raise ValueError("Unexpected number of columns in the angular velocity input file.")
        
        # Import location data

        location_data = np.loadtxt(str(self.data_path / (self.workpiece_name + '_simulated_data_export_location.txt')), dtype=float)

        if location_data.shape[1] == 6:
            self.array_location = location_data[:, :3].reshape(-1, 3)
            self.array_pre_impulse_location = location_data[:, 3:].reshape(-1, 3)
        else:    
            raise ValueError("Unexpected number of columns in the location input file.")
        
    # main function to find the poses of the workpiece
    def find_poses_main(self):

        if self.mode == 0:
            self.array_quaternion_blend = self._find_poses_torge(self.array_quaternion_blend)

            self.plot_mesh_visualization(self.array_quaternion_blend)
            self.plot_frequency_histogram(self.array_quaternion_blend)
            self.plot_pose_pie_chart(self.array_quaternion_blend)
                
        elif self.mode == 1:
            self.array_quaternion_blend = self._find_poses_quat(self.array_quaternion_blend)

            self.plot_mesh_visualization(self.array_quaternion_blend)
            self.plot_frequency_histogram(self.array_quaternion_blend)
            self.plot_pose_pie_chart(self.array_quaternion_blend)

        elif self.mode == 2:
            concatenated_quaternions = np.vstack((self.array_quaternion_blend, self.array_pre_impulse_quaternion_blend))
            print(self.array_pre_impulse_quaternion_blend)
            concatenated_quaternions = self._find_poses_quat(concatenated_quaternions)
            

            self.array_quaternion_blend = concatenated_quaternions[:self.simulation_number]
            self.array_pre_impulse_quaternion_blend = concatenated_quaternions[self.simulation_number:]

            # Call the function to write the modified quaternions to CSV files
            self.write_modified_quaternions_to_csv()

            self._find_simulation_outcomes()
            self._find_sliding_distance()

            # Create a boolean index for successful outcomes with the same length as concatenated_quaternions
            successful_indices = self.simulation_outcomes == 0

            if np.any(successful_indices):
                self.plot_mesh_visualization(concatenated_quaternions)
                self.plot_frequency_histogram(self.array_quaternion_blend[successful_indices, 4])
                self.plot_frequency_histogram(self.array_pre_impulse_quaternion_blend[successful_indices, 4])

            self.plot_simulation_outcome_pie_chart()

        elif self.mode == 3:

            self._find_simulation_outcomes()
            self._find_sliding_distance()

        else:
            raise ValueError("Invalid mode specified.")
    
    # Returns the frequency of the different outcomes  
    def get_simulation_outcome_frequency(self):  
        simulation_frequency = np.array([np.sum(self.simulation_outcomes == i) for i in range(5)])
        return simulation_frequency

    # Returns the sliding distance
    def get_sliding_distance_average(self):
        if np.sum(self.simulation_outcomes == 0) == 0:
            return 0
        return np.mean(abs(self.sliding_distance_array[self.simulation_outcomes == 0]))
    
    # Write the modified quaternion arrays to CSV files
    def write_modified_quaternions_to_csv(self):
        # Define file paths
        quaternion_blend_file = self.data_path / (self.workpiece_name + '_modified_quaternion_blend.csv')
        pre_impulse_quaternion_blend_file = self.data_path / (self.workpiece_name + '_modified_pre_impulse_quaternion_blend.csv')

        # Write array_quaternion_blend to CSV
        with open(quaternion_blend_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['w', 'x', 'y', 'z', 'pose'])
            writer.writerows(self.array_quaternion_blend)

        # Write array_pre_impulse_quaternion_blend to CSV
        with open(pre_impulse_quaternion_blend_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['w', 'x', 'y', 'z', 'pose'])
            writer.writerows(self.array_pre_impulse_quaternion_blend)

    # Check if the reorientation occured and classify the outcome
    def _find_simulation_outcomes(self):
        for i in range(self.array_location.shape[0]):
            if self.array_location[i, 2] >= 0:
                if max(abs(self.array_angular_velocity[i,:])) < 0.1:
                    if self._is_different_pose(self.array_quaternion_blend[i, :4], self.array_pre_impulse_quaternion_blend[i, :4], threshold=45):
                        if self._is_different_pose_x(self.array_quaternion_blend[i, :4], self.array_pre_impulse_quaternion_blend[i, :4], threshold=45):
                            self.simulation_outcomes[i] = 0 # Successfully reoriented
                        else:
                            self.simulation_outcomes[i] = 1 # Unsuccessfully reoriented
                    else:
                        self.simulation_outcomes[i] = 2 # Not reoriented
                else:
                    self.simulation_outcomes[i] = 3 # Workpiece is not settled
            else:
                self.simulation_outcomes[i] = 4 # Workpiece fell off the slide

    # This function calculates the sliding distance of sucessful reorientations
    def _find_sliding_distance(self):
        for i in range(self.simulation_outcomes.shape[0]):
            if self.simulation_outcomes[i] == 0:
                self.sliding_distance_array[i] = self.array_location[i, 1] - self.array_pre_impulse_location[i, 1]
            else:
                self.sliding_distance_array[i] = 0

    # This function classifies pose of workpieces according to their orientation determined in quaternion space, then converts to euler
    # (original function from Torge, only works for workpieces landing on a plane)
    def _find_poses_torge(self, array_quaternion_pose):

        n = 1
        eps = 0.3  # Arbitrarily determined thresholding parameter to separate regions of rotation into poses

        for i in range(len(array_quaternion_pose)):
            if array_quaternion_pose[i, 4] == 0:
                array_quaternion_pose[i, 4] = n

                x = array_quaternion_pose[i, :4]  # KS1 - Quaternion (w, x, y, z)
                
                for j in range(i+1, len(array_quaternion_pose)):
                    y = array_quaternion_pose[j, :4]  # KS2 - Quaternion (w, x, y, z)
                    
                    # Normalize the quaternions
                    x = self._normalize_quat(x)
                    y = self._normalize_quat(y)
                    
                    # z quaternion = _quat_conjugate(x) * y
                    z = self._quat_multiply(self._quat_conjugate(x), y)
                    
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
                    rotm_x = self._quat_to_rot_matrix(x)
                    rotm_y = self._quat_to_rot_matrix(y)
                    rotm_z = self._quat_to_rot_matrix(z)
                    
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

    # This function classifies pose of workpieces according to their orientation determined in quaternion space
    def _find_poses_quat(self, array_quaternion_pose):

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

                        if array_quaternion_pose[j, 4] == 0.0:  # If pose not classified
                            if not(self._is_different_pose(quat_i, quat_j, rot_diff_threshold)):
                                array_quaternion_pose[j, 4] = n  # Assign the same cluster label
                                #print(n)
                                #print(f"same pose at j {j}")
                            #else:
                                #print(f"different pose at j {j}")

                array_quaternion_pose[i, 4] = n
                n += 1  # Move to the next cluster label
        return array_quaternion_pose
    
    # helper function to determine if the pose is different
    def _is_different_pose(self, quat1, quat2, threshold=45):

        print("quat1", quat1)
        print("quat2", quat2)
        q2_inv = self._quat_conjugate(quat2)
        print("q2_inv",q2_inv)
        q_diff = tf_transformations.quaternion_multiply(quat1, q2_inv)
        q_diff = self._normalize_quat(q_diff)
        print("q_diff",q_diff)
        euler = tf_transformations.euler_from_quaternion([q_diff[0], q_diff[1], q_diff[2], q_diff[3]])
        print("euler",euler)
        # Normalize the quaternions
        quat1 = self._normalize_quat(quat1)
        quat2 = self._normalize_quat(quat2)
        print(quat1)
        print(quat2)
        # Calculate the distance between the two quaternions
        quat_diff = np.abs(self._quat_multiply(quat1, self._quat_conjugate(quat2)))
        print("quat_diff_Dasha", quat_diff)
        euler_dasha = tf_transformations.euler_from_quaternion([quat_diff[1], quat_diff[2], quat_diff[3], quat_diff[0]])
        print("euler_dasha", euler_dasha)

        # The scalar component of the quaternion is closer to 1 if the rotation is small and vice versa
        # Therefore if the rotation is smaller than the threshold the scalar component will be bigger than
        # the cosine of the threshold
        return quat_diff[0] < np.cos((threshold * np.pi) / 360) # check the scalar component of the quaternion
        #return np.abs(quat_diff[1]) > np.sin((threshold * np.pi) / 360)
    
    # helper function to determine if the pose is different around the x axis only
    def _is_different_pose_x(self, quat1, quat2, threshold=45):
        # Normalize the quaternions
        quat1 = self._normalize_quat(quat1)
        quat2 = self._normalize_quat(quat2)

        # Calculate the distance between the two quaternions
        quat_diff = np.abs(self._quat_multiply(quat1, self._quat_conjugate(quat2)))

        # The scalar component of the quaternion is closer to 1 if the rotation is small and vice versa
        # Therefore if the rotation is smaller than the threshold the scalar component will be bigger than
        # the cosine of the threshold
        return np.abs(quat_diff[1]) > np.sin((threshold * np.pi) / 360)

    # helper function to convert euler to rotational matricies
    @staticmethod
    def _euler_to_rot_matrix(ax, ay, az):
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
    
        # Quaternion operations that the find_poses function requres
    
    #Returns the conjugate of a quaternion
    @staticmethod
    def _quat_conjugate(quat):
        return np.array([quat[0], -quat[1], -quat[2], -quat[3]])

    #Multiplies two quaternions with the same behaviour as MATLAB
    @staticmethod
    def _quat_multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return np.array([w, x, y, z])

    #Converts a quaternion to a rotation matrix MANUALLY
    @staticmethod
    def _quat_to_rot_matrix(quat):
        w, x, y, z = quat
        
        # Compute the rotation matrix elements
        R = np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])

        return R
    
    # Normalizes a quaternion
    @staticmethod
    def _normalize_quat(quat):
        return quat / np.linalg.norm(quat)
    
    # This function plots the mesh of the workpiece in different orientations
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

                # Compute rotation matrix once for the pose
                rotm = self._quat_to_rot_matrix(quat)

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
    
    # This function plots a histogram of the frequency of the determined poses in the simulation and colours the bars based on stability
    def plot_frequency_histogram(self,array_quaternion_pose):
        total_poses = int(np.max(array_quaternion_pose))
        stable_poses_frequency, bin_edges = np.histogram(array_quaternion_pose, bins=np.arange(1, total_poses + 2))

        plt.figure()
        plt.bar(bin_edges[:-1], stable_poses_frequency, width=1, color='b', edgecolor='k', alpha=0.7)
        plt.xlabel('Pose')
        plt.ylabel('Frequency')
        plt.title('Pose Frequency Histogram')
        plt.show()

    # This function plots a pie chart of the frequency of the determined poses in the simulation
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
    
    # This function plots a pie chart of the re-orientation rate of the workpiece, the only sucessful outcome is coloured green
    def plot_simulation_outcome_pie_chart(self):
        # Calculate the counts for each outcome
        successful_reoriented = np.sum(self.simulation_outcomes == 0)
        unsuccessful_reoriented = np.sum(self.simulation_outcomes == 1)
        not_reoriented = np.sum(self.simulation_outcomes == 2)
        not_stable = np.sum(self.simulation_outcomes == 3)
        fell_off_slide = np.sum(self.simulation_outcomes == 4)

        # Data for the pie chart
        sizes = [successful_reoriented, unsuccessful_reoriented, not_reoriented, not_stable, fell_off_slide]
        labels = ['Successfully Re-oriented', 'Unsuccessfully Re-oriented', 'Not Reoriented', 'Not Settled', 'Fell Off Slide']
        colors = ['#99ff99', '#ff9999', '#ff6666', '#ff3333', '#ff0000']  # Green for success, shades of red for failures

        # Filter out categories with 0 results using list comprehension
        filtered_data = [(size, label, color) for size, label, color in zip(sizes, labels, colors) if size > 0]
        
        # Unzip the filtered data
        sizes, labels, colors = zip(*filtered_data) if filtered_data else ([], [], [])

        # Plot the pie chart
        plt.figure(figsize=(10, 6))
        plt.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', startangle=140)
        plt.title('Simulation Outcomes')
        plt.legend(labels, loc='center left', bbox_to_anchor=(1.2, 0.5))
        plt.show()
