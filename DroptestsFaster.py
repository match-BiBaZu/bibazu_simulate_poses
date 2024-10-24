from pathlib import Path
import pybullet as p
import pybullet_data
import time
import random
import numpy as np # numpy HAS to be 1.26.4 at the latest for compatibility with PyBullet
import trimesh as tm

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
        self.Alpha = 0.0

        # This is the tilt angle of the slide perpendicular to it's 'sliding' axis
        self.Beta = 0.0

        # This is the feeding speed of the workpiece before it begins to slide down the surface
        self.workpiece_feed_speed = 5.0

        # This is the offset of the hitpoint on one of the slide surfaces parallel to the sliding axis from the input end of the surface
        self.hitpoint_offset_parallel = 0.0

        # This is the offset of the nozzle on one of the slide surfaces parallel to the sliding axis from the input end of the surface
        self.nozzle_offset_parallel = 0.0
        
        # This is the offset of the nozzle on one of the slide surfaces perpendicular from the sliding axis
        self.nozzle_offset_perpendicular = 0.0

        # This is the impulse force applied by the nozzle to the workpiece
        self.nozzle_impulse_force = 0.0

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
    def is_over_location(workpiece_hitpoint, nozzle_position , impulse_error_threshold=0.1):
        # Check if the y coordinates are within the impulse_error_threshold
        distance_y = abs(workpiece_hitpoint[1] - nozzle_position [1])
        return distance_y < impulse_error_threshold

    def drop_tests(self):
        impulse_threshold = 0.1  # Define the impulse threshold for stopping
        simulation_steps = 1500 # Define the maximum number of simulation steps
        current_simulation = 1  # Initialize the simulation number

        # Initialize PyBullet and set up physics simulation
        p.connect(p.GUI)  # Use p.DIRECT for non-GUI mode
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # PyBullet's internal data path
        p.setGravity(0, 0, -9.81)  # Set gravity in the simulation

        # Initialise the surface
        #--------------------------------------------------------------------------
        surface_mesh = tm.load(str(self.surface_path / (self.surface_name + '.obj')))

        # Find the length of the bounding box (axis-aligned) along each axis
        axis_lengths = surface_mesh.bounding_box.extents

        # Find the longest axis length from the bounding box
        surface_slide_length = max(axis_lengths)

        # Find the cross section length of the surface (assuming it is a square)
        surface_cross_section_size = min(axis_lengths)

        # Calculate the lengths of the unique edges of the mesh and find the shortest one
        surface_thickness = np.min(np.linalg.norm(surface_mesh.vertices[surface_mesh.edges_unique][:, 0] - surface_mesh.vertices[surface_mesh.edges_unique][:, 1], axis=1))
        
        # Determine the end point of the sliding action that the workpiece will reach on the surface
        surface_end_point = (surface_slide_length/2)*np.cos(np.radians(self.Alpha))
        
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
        rotation_y = p.getQuaternionFromEuler([0, np.radians(90-self.Beta), 0])   # Rotation around Y-axis

        # Combine both quaternions by multiplying them
        surface_rotation = p.multiplyTransforms([0, 0, 0], rotation_x, [0, 0, 0], rotation_y)[1]

        # Apply the combined rotation to the object in quaternion form to avoid gimbal lock
        p.resetBasePositionAndOrientation(surface_id, [0, 0, 0], surface_rotation)
        
        # Load the workpiece model
        #--------------------------------------------------------------------------
        # Calculate the starting position of the workpiece 1 meter above the surface
        workpiece_start_x = self.nozzle_offset_perpendicular * np.cos(np.radians(self.Beta))
        workpiece_start_y = (surface_slide_length/2 - 0.2)*np.cos(np.radians(self.Alpha))
        workpiece_start_z = ((surface_slide_length/2 - 0.2)*np.sin(np.radians(self.Alpha)) + 
                             self.nozzle_offset_perpendicular * np.sin(np.radians(self.Beta)) + 0.5)
        workpiece_start_pos = [workpiece_start_x, workpiece_start_y, workpiece_start_z]  # Start 1 meter above the surface

        # Random workpiece orientations defined in each iteration
        workpiece_start_orientation = self.rand_orientation()

        # Create a mesh for the workpiece
        workpiece_mesh = tm.load(str(self.workpiece_path / (self.workpiece_name + '.obj')))

        # Find the geometric center of the workpiece
        workpiece_geometric_center = workpiece_mesh.centroid

        # Find the mass of the workpiece
        workpiece_mass = workpiece_mesh.volume * 1100 # Density of Aqua 8K V4 Resin is 1100 kg/m³
        
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
            baseMass=1.0,
            baseCollisionShapeIndex=workpiece_collision_id,
            baseVisualShapeIndex=workpiece_vis_shape_id,
            basePosition=workpiece_start_pos,
            baseOrientation=workpiece_start_orientation,
            baseInertialFramePosition=workpiece_geometric_center,
        )

        # initialise the nozzle
        #--------------------------------------------------------------------------
        # Determine the location of the nozzle on the surface
        nozzle_position_x = self.nozzle_offset_perpendicular * np.cos(np.radians(self.Beta))
        nozzle_position_y = ((surface_slide_length - self.nozzle_offset_parallel)/2)*np.cos(np.radians(self.Alpha) + 
            self.nozzle_offset_perpendicular * np.sin(np.radians(self.Beta)))
        nozzle_position_z = (((surface_slide_length - self.nozzle_offset_parallel) / 2) * np.sin(np.radians(self.Alpha)))

        nozzle_position = [nozzle_position_x, nozzle_position_y, nozzle_position_z]

        workpiece_hitpoint = [workpiece_start_x, workpiece_start_y + self.hitpoint_offset_parallel, workpiece_start_z]
        

        # Calculate the nozzle direction considering the tilt angles Alpha and Beta
        nozzle_direction = [
            self.nozzle_impulse_force * np.sin(np.radians(self.Beta)),
            self.nozzle_impulse_force * np.cos(np.radians(self.Alpha)) * np.cos(np.radians(self.Beta)),
            self.nozzle_impulse_force * np.sin(np.radians(self.Alpha))
        ]

        impulse_error_threshold = 0.1  # Threshold for checking when CoG passes over the point

        nozzle_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1])  # Red sphere
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
    
        # Update the COM marker position in each simulation step
        def update_com_marker():
            com_position, _ = p.getBasePositionAndOrientation(workpiece_id)
            p.resetBasePositionAndOrientation(com_marker, com_position, [0, 0, 0, 1])
        # ----------------------------------------------------------------------------

        # Set dynamic properties for the plane
        p.changeDynamics(surface_id, -1, restitution=0.05, lateralFriction=0.04, linearDamping=0.04, angularDamping=0.1)

        # Set dynamic properties for the workpiece mass is in kg/m³ and the density of Aqua 8K V4 Resin is 1100 kg/m³
        p.changeDynamics(workpiece_id, -1, mass=workpiece_mass, restitution=0.8, lateralFriction=0.5, linearDamping=0.04, angularDamping=0.1)

        # File paths for simulated data
        workpiece_data_path = self.data_path / (self.workpiece_name + '_simulated_data.txt')
        workpiece_location_path = self.data_path / (self.workpiece_name + '_simulated_data_export_matlab_location.txt')
        workpiece_quaternion_path = self.data_path / (self.workpiece_name + '_simulated_data_export_matlab_quaternion.txt')

        # Initialize matricies to store simulation data
        matrix_location = []
        matrix_rotation_quaternion = []

        # Clear the text files before starting (open in 'w' mode)
        with open(workpiece_location_path, 'w') as loc_file, \
            open(workpiece_quaternion_path, 'w') as quat_file:
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
                for step in range(simulation_steps):  # Maximum number of simulation steps
                    p.stepSimulation()  # Step the simulation forward

                    #update_com_marker()

                    # Get the workpiece's current position and orientation
                    position, bullet_orientation = p.getBasePositionAndOrientation(workpiece_id)

                    # update the workpiece hitpoint location
                    workpiece_hitpoint = [position[0], position[1] + self.hitpoint_offset_parallel, position[2]]

                    # Apply an orientation to negate Alpha and Beta
                    # Create a quaternion to negate Alpha and Beta rotations
                    negating_rotation = p.invertTransform([0, 0, 0], surface_rotation)[1] 

                    # Use PyBullet's multiplyTransforms to multiply quaternions
                    _, bullet_orientation = p.multiplyTransforms([0, 0, 0], negating_rotation, [0, 0, 0], bullet_orientation)

                    #Change quaternion ordering from w,x,y,z to x,y,z,w to match blender model outputs
                    blender_orientation = (bullet_orientation[1], bullet_orientation[2], bullet_orientation[3], bullet_orientation[0])

                    euler_orientation = p.getEulerFromQuaternion(bullet_orientation)

                    #Change euler orientation to match blender model outputs
                    euler_orientation =[np.degrees(euler_orientation[0])-90, np.degrees(euler_orientation[1]), np.degrees(euler_orientation[2])]
                    
                    # Get linear and angular velocity to detect stopping condition
                    linear_velocity, angular_velocity = p.getBaseVelocity(workpiece_id)
                    angular_velocity_magnitude = np.linalg.norm(angular_velocity)

                    # Get contact points between the plane and the workpiece
                    contact_points = p.getContactPoints(bodyA=surface_id, bodyB=workpiece_id)

                    print(f"Number of Contact Points: {len(contact_points)}")
                    
                    # Slow down the simulation to match real-time (optional)
                    time.sleep(1 / 240.)

                    # Apply an initial velocity to the workpiece once the workpiece has made contact with the surface
                    #if len(contact_points) > 2 and apply_velocity == True:
                        
                    #    self.set_workpiece_velocity(workpiece_id, self.workpiece_feed_speed, [0,-np.cos(np.radians(self.Alpha)),np.sin(np.radians(self.Alpha))])
                    #    apply_velocity = False

                    # Check if CoG is over impulse location
                    if self.is_over_location(workpiece_hitpoint, nozzle_position , impulse_error_threshold):
                        # Apply impulse force
                        p.applyExternalForce(workpiece_id, -1, nozzle_direction, nozzle_position , p.WORLD_FRAME)
                        print("impulse applied")

                    # Stop the simulation when the workpiece reaches the end of the slide
                    if position[1] < -surface_end_point:
                       print(f"Object '{self.workpiece_name}' reached equilibrium at step {step} with contact")
                       break
   
                print(f"Simulation {current_simulation}, Step {step}")
                current_simulation+= 1
                matrix_location.append(position)
                matrix_rotation_quaternion.append(blender_orientation)

                # Write iteration data to workpiece_data
                workpiece_data.writelines(f"\nITERATION: {current_simulation}\n")
                workpiece_data.writelines(f"LAST STEP: {step}\n")
                workpiece_data.writelines(f"Simulated Location (XYZ) [mm]: {position[0]}, {position[1]}, {position[2]}\n")
                workpiece_data.writelines(f"Simulated Rotation Euler (XYZ) [°]: {euler_orientation[0]}, {euler_orientation[1]}, {euler_orientation[2]}\n")
                workpiece_data.writelines(f"Simulated Rotation Quaternion (w, x, y, z): {bullet_orientation[0]}, {bullet_orientation[1]}, {bullet_orientation[2]}, {bullet_orientation[3]}\n")
            
            # Save the simulation data
            np.savetxt(workpiece_location_path, np.array(matrix_location), delimiter='\t')
            np.savetxt(workpiece_quaternion_path, np.array(matrix_rotation_quaternion), delimiter='\t')


        # Disconnect from PyBullet
        p.disconnect()


