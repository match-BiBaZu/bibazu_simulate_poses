from pathlib import Path
import pybullet as p
import pybullet_data
import time
import random
import numpy as np # numpy HAS to be 1.26.4 at the latest for compatibility with PyBullet
import trimesh as tm
import matplotlib.pyplot as plt

class DroptestsFaster:
    def __init__(self):
        # This is the current name of the workpiece
        self.workpiece_name = 'r2d2'

        # This is the current name of the surface
        self.surface_name = 'Plane'

        # This is the total number of simulations - usually 1000
        self.simulation_number = 1000

        #--------------------------------------------------------------------------
        # MODIFIABLE SURFACE AND WORKPIECE PARAMETERS:

        # This is the tilt angle of the slide along it's 'sliding' axis
        self.Alpha = 20.0

        # This is the tilt angle of the slide perpendicular to it's 'sliding' axis
        self.Beta = 45.0

        # This is the feeding speed of the workpiece before it begins to slide down the surface
        self.workpiece_feed_speed = 5.0

        # This is the offset of the hitpoint on one of the slide surfaces parallel to the sliding axis from the input end of the surface
        self.hitpoint_offset_parallel = 0.0

        # This is the offset of the nozzle on one of the slide surfaces parallel to the sliding axis from the input end of the surface
        self.nozzle_offset_parallel = 0.1
        
        # This is the offset of the nozzle on one of the slide surfaces perpendicular from the sliding axis
        self.nozzle_offset_perpendicular = 0.02

        # This is the impulse force applied by the nozzle to the workpiece
        self.nozzle_impulse_force = 0.1

        #--------------------------------------------------------------------------
        
        # This is the current file path of the data stored relative to the script
        self.data_path = Path(__file__).parent / 'SimulationData' / 'MHI_Data'

        # This is the current file path of the workpiece stls relative to the script
        self.workpiece_path =  Path(__file__).parent / 'Workpieces'

        # This is the current file path of the workpiece stls relative to the script
        self.surface_path =  Path(__file__).parent / 'Surfaces'


    # Overrides the parameters defined in init, is done this way as you can have a flexible number of arguments
    def config(self, **kwargs):

        if 'workpiece_name' in kwargs:
            self.workpiece_name = kwargs['workpiece_name']
        if 'surface_name' in kwargs:
            self.surface_name = kwargs['surface_name']
        if 'simulation_number' in kwargs:
            self.simulation_number = kwargs['simulation_number']
        if 'Alpha' in kwargs:
            self.Alpha = kwargs['Alpha']
        if 'Beta' in kwargs:
            self.Beta = kwargs['Beta']
        if 'workpiece_feed_speed' in kwargs:
            self.workpiece_feed_speed = kwargs['workpiece_feed_speed']
        if 'data_path' in kwargs:
            self.data_path = kwargs['data_path']
        if 'workpiece_path' in kwargs:
            self.workpiece_path = kwargs['workpiece_path']
        if 'surface_path' in kwargs:
            self.surface_path = kwargs['surface_path']
        if 'hitpoint_offset_parallel' in kwargs:
            self.hitpoint_offset_parallel = kwargs['hitpoint_offset_parallel']
        if 'nozzle_offset_parallel' in kwargs:
            self.nozzle_offset_parallel = kwargs['nozzle_offset_parallel']
        if 'nozzle_offset_perpendicular' in kwargs:
            self.nozzle_offset_perpendicular = kwargs['nozzle_offset_perpendicular']
        if 'nozzle_impulse_force' in kwargs:
            self.nozzle_impulse_force = kwargs['nozzle_impulse_force']

    # Function to set initial velocity after contact detection
    def set_workpiece_velocity(self, workpiece_id, magnitude, direction):
        norm_direction = np.linalg.norm(direction)
        if norm_direction == 0:
            raise ValueError("Direction vector cannot be zero.")
        
        normalized_direction = np.array(direction) / norm_direction
        initial_velocity = normalized_direction * magnitude
        p.resetBaseVelocity(workpiece_id, linearVelocity=initial_velocity, angularVelocity=(0,0,0))

    # This function is used to initialise the workpiece in a random orientation
    @staticmethod
    def rand_orientation():
        # Random rotations for workpiece
        rand_rot_x_W = round(random.uniform(0, 2 * np.pi), 5)
        rand_rot_y_W = round(random.uniform(0, 2 * np.pi), 5)
        rand_rot_z_W = round(random.uniform(0, 2 * np.pi), 5)

        workpiece_start_orientation = p.getQuaternionFromEuler([rand_rot_x_W , rand_rot_y_W, rand_rot_z_W])
        return workpiece_start_orientation

    @staticmethod
    def is_over_location(workpiece_hitpoint, nozzle_position):

        position_error_threshold = 0.01
        # Calculate the Euclidean distance between the workpiece hitpoint and the nozzle position
        distance = workpiece_hitpoint[1] - nozzle_position[1]
        # print(f"Distance between workpiece hitpoint and nozzle position: {distance}")
        return abs(distance) < position_error_threshold

    def drop_tests(self):
        simulation_steps = 1500 # Define the maximum number of simulation steps
        current_simulation = 1  # Initialize the simulation number

        # Initialize PyBullet and set up physics simulation
        p.connect(p.DIRECT)  # Use p.DIRECT for non-GUI mode
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # PyBullet's internal data path
        p.setGravity(0, 0, -9.81)  # Set gravity in the simulation

        # Initialise the surface
        #--------------------------------------------------------------------------
        surface_mesh = tm.load(str(self.surface_path / (self.surface_name + '.obj')))

        # Find the length of the bounding box (axis-aligned) along each axis
        surface_lengths = surface_mesh.bounding_box.extents

        # Find the longest axis length from the bounding box
        surface_slide_length = max(surface_lengths)
        
        # Create a convex hull collision shape for the surface
        surface_collision_id = p.createCollisionShape(
            shapeType=p.GEOM_MESH,          # Specify that this is a mesh
            fileName=str(self.surface_path / (self.surface_name + '.obj')),  # Path to your OBJ file
            flags= p.GEOM_FORCE_CONCAVE_TRIMESH | p.GEOM_CONCAVE_INTERNAL_EDGE # Use concave hull shape for collision
        )

        # Create a visual shape for the surface (optional, for rendering in the GUI)
        surface_vis_shape_id = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=str(self.surface_path / (self.surface_name + '.obj'))
        )
    
        # Create a multi-body with the collision and visual shapes for the surface
        surface_id = p.createMultiBody(
            baseMass=0.0,  # Static object
            baseCollisionShapeIndex=surface_collision_id,
            baseVisualShapeIndex=surface_vis_shape_id,
            basePosition= [0,0,0],  # Place the surface at the origin (which is the geometric center)  
            baseOrientation= [0,0,0,1],  # first no rotation only translation to the center of the origin
            baseInertialFramePosition=[0,0,0],
        )

        # Create separate quaternions for rotation around the X and Y axes
        rotation_x = p.getQuaternionFromEuler([np.radians(self.Alpha), 0, 0])  # Rotation around X-axis
        rotation_y = p.getQuaternionFromEuler([0, -np.radians(self.Beta), 0])   # Rotation around Y-axis

        # Combine both quaternions by multiplying them
        surface_rotation = p.multiplyTransforms([0, 0, 0], rotation_x, [0, 0, 0], rotation_y)[1]

        # Apply the combined rotation to the object in quaternion form to avoid gimbal lock
        p.resetBasePositionAndOrientation(surface_id, [0, 0, 0], surface_rotation)
        
        # Load the workpiece model
        #--------------------------------------------------------------------------
        # Calculate the starting position of the workpiece 1 meter above the surface
        # Define the local starting position of the workpiece relative to the surface
        local_workpiece_start_pos = [0.03, (surface_slide_length / 2 - 0.03), 0.03 ]

        # Apply the surface rotation to the local starting position
        workpiece_start_pos, _ = p.multiplyTransforms([0, 0, 0], surface_rotation, local_workpiece_start_pos, [0, 0, 0, 1] )

        # Random workpiece orientations defined in each iteration
        workpiece_start_orientation = self.rand_orientation()

        # Create a mesh for the workpiece
        workpiece_mesh = tm.load(str(self.workpiece_path / (self.workpiece_name + '.obj')))

        # Find the geometric center of the workpiece
        workpiece_geometric_center = workpiece_mesh.centroid

        # Find the mass of the workpiece
        workpiece_mass = workpiece_mesh.volume * 1100 # Density of Aqua 8K V4 Resin is 1100 kg/m³
        #print(f"Workpiece mass: {workpiece_mass} kg")

        # Create the collision shape using the obj file
        # Create a convex hull collision shape for the workpiece
        workpiece_collision_id = p.createCollisionShape(
            shapeType=p.GEOM_MESH,          # Specify that this is a mesh
            fileName=str(self.workpiece_path / (self.workpiece_name + '.obj')),  # Path to your OBJ file
        )

        # Create a visual shape for the workpiece (optional, for rendering in the GUI)
        workpiece_vis_shape_id = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=str(self.workpiece_path / (self.workpiece_name + '.obj'))
        )

        # Create a multi-body with the collision and visual shapes for the workpiece
        workpiece_id = p.createMultiBody(
            baseMass=workpiece_mass,
            baseCollisionShapeIndex=workpiece_collision_id,
            baseVisualShapeIndex=workpiece_vis_shape_id,
            basePosition=workpiece_start_pos,
            baseOrientation=workpiece_start_orientation,
            baseInertialFramePosition=workpiece_geometric_center,
        )

        # initialise the nozzle
        #--------------------------------------------------------------------------
        # Determine the location of the nozzle on the surface

        # Define the local position of the nozzle relative to the surface
        local_nozzle_position = [self.nozzle_offset_perpendicular,(surface_slide_length/ 2- self.nozzle_offset_parallel),0]

        # Apply the surface rotation to the local nozzle position
        nozzle_position, _ = p.multiplyTransforms([0, 0, 0], surface_rotation, local_nozzle_position, [0, 0, 0, 1])

        workpiece_hitpoint = [workpiece_start_pos[0], workpiece_start_pos[1] + self.hitpoint_offset_parallel, workpiece_start_pos[2]]

        # Convert the quaternion result to a rotation matrix
        rotation_matrix = p.getMatrixFromQuaternion(surface_rotation)

        # Extract the Z-axis vector from the rotation matrix
        nozzle_direction = [-rotation_matrix[6], -rotation_matrix[7], rotation_matrix[8]] # set the values to negative as the rotations are in the opposite direction

        # Normalize the nozzle direction vector
        nozzle_direction = np.array(nozzle_direction) / np.linalg.norm(nozzle_direction)

        nozzle_force = [self.nozzle_impulse_force * nozzle_direction[0], self.nozzle_impulse_force * nozzle_direction[1], self.nozzle_impulse_force * nozzle_direction[2]]

        nozzle_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.005, rgbaColor=[1, 0, 0, 1])  # Red sphere
        
        nozzle_marker = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=nozzle_visual_shape,
            basePosition=nozzle_position 
        )
        # DEBUGGING VISUALIZATIONS
        # ----------------------------------------------------------------------------   

        # Enable visualizing collision shapes
        #p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1)

        # Enable the GUI and other visualizations if needed
        #p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

        # Draw a sphere at the adjusted COM to visualize it
        #com_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1])  # Red sphere
        #com_marker = p.createMultiBody(
        #    baseMass=0,
        #    baseVisualShapeIndex=com_visual_shape,
        #    basePosition=[0, nozzle_position_y,nozzle_position_z]
        #)

        # Add the vector as a debug line in PyBullet
        p.addUserDebugLine(
            nozzle_position,
            [nozzle_position[i] + nozzle_direction[i] for i in range(3)],  # End point of the vector
            lineColorRGB=[0.1, 0, 0],  # Color of the vector (red)
            lineWidth=3,             # Line width
            lifeTime=0               # Duration the line will persist, 0 for infinite
        )
        # Update the COM marker position in each simulation step
        def update_com_marker():
            com_position, _ = p.getBasePositionAndOrientation(workpiece_id)
            p.resetBasePositionAndOrientation(com_marker, com_position, [0, 0, 0, 1])
        # ----------------------------------------------------------------------------

        # Set dynamic properties for the plane
        p.changeDynamics(surface_id, -1, restitution=0.05, lateralFriction=0.04, linearDamping=0.04, angularDamping=0.1)

        # Set dynamic properties for the workpiece mass is in kg/m³ and the density of Aqua 8K V4 Resin is 1100 kg/m³
        p.changeDynamics(workpiece_id, -1, mass=workpiece_mass, restitution=0.8, lateralFriction=0.5, linearDamping=0.5, angularDamping=0.1)

        # File paths for simulated data
        workpiece_data_path = self.data_path / (self.workpiece_name + '_simulated_data.txt')
        workpiece_angular_velocity_path = self.data_path / (self.workpiece_name + '_simulated_data_export_angular_velocity.txt')
        workpiece_contact_points_path = self.data_path / (self.workpiece_name + '_simulated_data_export_contact_points.txt')
        workpiece_quaternion_path = self.data_path / (self.workpiece_name + '_simulated_data_export_quaternion.txt')
        workpiece_location_path = self.data_path / (self.workpiece_name + '_simulated_data_export_location.txt')

        # Initialize matricies to store simulation data
        matrix_rotation_quaternion = []
        matrix_angular_velocity = []
        matrix_contact_points = []
        matrix_location = []

        pre_impulse_angular_velocity = np.zeros(3)
        pre_impulse_orientation = np.ones(4) * 1000000
        pre_impulse_contact_points = np.ones(2) * 1000000
        pre_impulse_location = np.ones(3) * 1000000

        angular_velocity_buffer = np.zeros(3)
        quaternion_buffer = np.zeros(4)

        # Clear the text files before starting (open in 'w' mode)
        with open(workpiece_angular_velocity_path, 'w') as ang_vel_file, \
            open(workpiece_contact_points_path, 'w') as contact_points_file, \
            open(workpiece_quaternion_path, 'w') as quat_file, \
            open(workpiece_location_path, 'w') as location_file :
            pass  # Just opening the files in 'w' mode will clear them

        # Open workpiece data file once
        with open(workpiece_data_path, 'w') as workpiece_data:
            workpiece_data.writelines("-------------------------------------------")
            workpiece_data.writelines('\nSimulated_Data_' + self.workpiece_name + '\n')
            workpiece_data.writelines("-------------------------------------------\n")

            while current_simulation < self.simulation_number + 1:

                # Random workpiece rotations defined in each iteration
                workpiece_start_orientation = self.rand_orientation()

                # Reset the position and orientation of the workpiece before each iteration
                p.resetBasePositionAndOrientation(workpiece_id, workpiece_start_pos, workpiece_start_orientation)
                # Apply an initial velocity to the workpiece after resetting its position and orientation
                p.resetBaseVelocity(workpiece_id, [0, -self.workpiece_feed_speed * np.cos(np.radians(self.Alpha)), -self.workpiece_feed_speed * np.sin(np.radians(self.Alpha))], [0, 0, 0])
                # Run the simulation
                impulse_applied = False

                for step in range(simulation_steps):  # Maximum number of simulation steps
                    p.stepSimulation()  # Step the simulation forward

                    #update_com_marker()

                    # Get the workpiece's current position and orientation
                    position, orientation = p.getBasePositionAndOrientation(workpiece_id)

                    # update the workpiece hitpoint location
                    workpiece_hitpoint = [position[0], position[1] + self.hitpoint_offset_parallel, position[2]]

                    # Apply an orientation to negate Alpha and Beta
                    # Create a quaternion to negate Alpha and Beta rotations
                    negating_rotation = p.invertTransform([0, 0, 0], surface_rotation)[1] 

                    # Use PyBullet's multiplyTransforms to multiply quaternions
                    workpiece_position, bullet_orientation = p.multiplyTransforms([0, 0, 0], negating_rotation, position, orientation)

                    #Change quaternion ordering from w,x,y,z to x,y,z,w to match blender model outputs
                    blender_orientation = (bullet_orientation[1], bullet_orientation[2], bullet_orientation[3], bullet_orientation[0])
                    
                    euler_orientation = p.getEulerFromQuaternion(bullet_orientation)

                    #Change euler orientation to match blender model outputs
                    euler_orientation =[np.degrees(euler_orientation[0])-90, np.degrees(euler_orientation[1]), np.degrees(euler_orientation[2])]
                    
                    # Get linear and angular velocity to detect stopping condition
                    linear_velocity, angular_velocity = p.getBaseVelocity(workpiece_id)

                    # Smooth the angular velocity using a moving average
                    angular_velocity_buffer = np.vstack((angular_velocity_buffer, angular_velocity))
                    quaternion_buffer = np.vstack((quaternion_buffer, blender_orientation))

                    if angular_velocity_buffer.shape[0] > 10:
                        angular_velocity_buffer = angular_velocity_buffer[-10:]
                        quaternion_buffer = quaternion_buffer[-10:]

                    angular_velocity_smoothed = np.mean(angular_velocity_buffer, axis=0)
                    #print(f"Angular Velocity Smoothed: {angular_velocity_smoothed}")
                    orientation_smoothed = np.mean(quaternion_buffer, axis=0)
                    #print(f"Orientation Smoothed: {orientation_smoothed}")
                    
                    # Get contact points between the plane and the workpiece
                    contact_points = p.getContactPoints(bodyA=surface_id, bodyB=workpiece_id)
                    
                    # Slow down the simulation to match real-time (optional)
                    #time.sleep(10 / 240.)

                    # Check if CoG is over impulse location
                    if self.is_over_location(workpiece_hitpoint, nozzle_position) and not impulse_applied:
                        pre_impulse_orientation = orientation_smoothed
                        #print(f"Workpiece pre impulse orientation: {pre_impulse_orientation}")

                        pre_impulse_angular_velocity = angular_velocity_smoothed
                        pre_impulse_location = workpiece_position
                        pre_impulse_contact_points = contact_points

                        workpiece_data.writelines(f"\nITERATION: {current_simulation}\n")
                        workpiece_data.writelines(f"IMPULSE APPLIED AT STEP: {step}\n")
                        workpiece_data.writelines(f"Simulated Location (XYZ) [mm]: {workpiece_position[0]}, {workpiece_position[1]}, {workpiece_position[2]}\n")
                        workpiece_data.writelines(f"Simulated Rotation Euler (XYZ) [°]: {euler_orientation[0]}, {euler_orientation[1]}, {euler_orientation[2]}\n")
                        workpiece_data.writelines(f"Simulated Number of Contact Points: {len(contact_points)}\n")
                        workpiece_data.writelines(f"Simulated Angular Velocity (XYZ) [rad/s]: {angular_velocity_smoothed[0]}, {angular_velocity_smoothed[1]}, {angular_velocity_smoothed[2]}\n")                           
                        workpiece_data.writelines(f"Simulated Rotation Quaternion (w, x, y, z): {orientation_smoothed[0]}, {orientation_smoothed[1]}, {orientation_smoothed[2]}, {orientation_smoothed[3]}\n")
                        
                        # Apply impulse force to the workpiece hit point
                        p.applyExternalForce(workpiece_id, -1, nozzle_force, nozzle_position , p.WORLD_FRAME)
                        #print("impulse applied")
                        impulse_applied = True
                    else:
                        # Ensure no force is applied when not over the location
                        p.applyExternalForce(workpiece_id, -1, [0, 0, 0], nozzle_position , p.WORLD_FRAME)

                    # Stop the simulation when the workpiece reaches equilibrium on the slide after impulse application
                    if impulse_applied and max(abs(angular_velocity_smoothed)) < 0.1 and len(contact_points) > 0:
                        #print(f"Object '{self.workpiece_name}' reached equilibrium at step {step} with contact")
                        break
   
                # print(f"Simulation {current_simulation}, Step {step}")
                # print(f"Angular Velocity at Step {step}: {max(abs(angular_velocity_smoothed))}")
                # print(f"Number of contact points at step {step}: {len(contact_points)}")
                current_simulation+= 1
                
                combined_orientation = np.concatenate((orientation_smoothed, pre_impulse_orientation))                            
                combined_angular_velocity = np.concatenate((angular_velocity_smoothed, pre_impulse_angular_velocity))
                combined_contact_points = [len(contact_points),len(pre_impulse_contact_points)]
                combined_location = np.concatenate((workpiece_position, pre_impulse_location))

                matrix_rotation_quaternion.append(combined_orientation)
                matrix_angular_velocity.append(combined_angular_velocity)
                matrix_contact_points.append(combined_contact_points)
                matrix_location.append(combined_location)

                # Write iteration data to workpiece_data
                workpiece_data.writelines(f"\nITERATION: {current_simulation}\n")
                workpiece_data.writelines(f"LAST STEP: {step}\n")
                workpiece_data.writelines(f"Simulated Location (XYZ) [mm]: {workpiece_position[0]}, {workpiece_position[1]}, {workpiece_position[2]}\n")
                workpiece_data.writelines(f"Simulated Rotation Euler (XYZ) [°]: {euler_orientation[0]}, {euler_orientation[1]}, {euler_orientation[2]}\n")
                workpiece_data.writelines(f"Simulated Number of Contact Points: {len(contact_points)}\n")
                workpiece_data.writelines(f"Simulated Angular Velocity (XYZ) [rad/s]: {angular_velocity[0]}, {angular_velocity[1]}, {angular_velocity[2]}\n")                           
                workpiece_data.writelines(f"Simulated Rotation Quaternion (w, x, y, z): {blender_orientation[0]}, {blender_orientation[1]}, {blender_orientation[2]}, {blender_orientation[3]}\n")
            
            # Save the simulation data
            np.savetxt(workpiece_angular_velocity_path, np.array(matrix_angular_velocity), delimiter='\t')
            np.savetxt(workpiece_contact_points_path, np.array(matrix_contact_points), delimiter='\t')
            np.savetxt(workpiece_quaternion_path, np.array(matrix_rotation_quaternion), delimiter='\t')
            np.savetxt(workpiece_location_path, np.array(matrix_location), delimiter='\t')

        # Disconnect from PyBullet
        p.disconnect()


