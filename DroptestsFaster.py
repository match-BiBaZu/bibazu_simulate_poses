from pathlib import Path
import pybullet as p
import pybullet_data
import time
import random
from math import pi, degrees, radians
import numpy as np # numpy HAS to be 1.26.4 at the latest for compatibility with PyBullet

class DroptestsFaster:
    def __init__(self):
        # This is the current name of the workpiece
        self.workpiece_name = 'r2d2'

        # This is the current name of the surface
        self.surface_name = 'Plane'

        # This is the total number of simulations - usually 1000
        self.simulation_number = 1000

        # This is the tilt angle of the slide along it's 'sliding' axis
        self.Alpha = 0.0

        # This is the tilt angle of the slide perpendicular to it's 'sliding' axis
        self.Beta = 0.0

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
        if 'data_path' in kwargs:
            self.data_path = kwargs['data_path']
        if 'workpiece_path' in kwargs:
            self.workpiece_path = kwargs['workpiece_path']
        if 'surface_path' in kwargs:
            self.surface_path = kwargs['surface_path']

    @staticmethod
    def calculate_geometric_center(obj_filepath):
        vertices = []

        # Open and read the .obj file
        with open(obj_filepath, 'r') as file:
            for line in file:
                # Check if the line starts with 'v', which indicates a vertex
                if line.startswith('v '):
                    # Split the line by spaces and extract the vertex coordinates
                    parts = line.split()
                    vertex = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
                    vertices.append(vertex)

        # Convert the list of vertices into a numpy array
        vertices = np.array(vertices)

        # Calculate the geometric center by averaging the vertex positions
        geometric_center = np.mean(vertices, axis=0)

        return geometric_center

    @staticmethod
    def rand_orientation():
        # Random rotations for workpiece
        rand_rot_x_W = round(random.uniform(0, 2 * pi), 5)
        rand_rot_y_W = round(random.uniform(0, 2 * pi), 5)
        rand_rot_z_W = round(random.uniform(0, 2 * pi), 5)

        workpiece_start_orientation = p.getQuaternionFromEuler([rand_rot_x_W , rand_rot_y_W, rand_rot_z_W])
        return workpiece_start_orientation
    
    def drop_tests(self):
        impulse_threshold = 0.005  # Define the impulse threshold for stopping
        simulation_steps = 1500 # Define the maximum number of simulation steps

        # Initialize PyBullet and set up physics simulation
        p.connect(p.GUI)  # Use p.DIRECT for non-GUI mode
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # PyBullet's internal data path
        p.setGravity(0, 0, -9.81)  # Set gravity in the simulation

        # Load the workpiece model (use a simple object for this example)
        workpiece_start_pos = [0, 0, 1]  # Start above the plane

        # Random workpiece orientations defined in each iteration
        workpiece_start_orientation = self.rand_orientation()

        # Find the geometric center of the workpiece
        workpiece_geometric_center = self.calculate_geometric_center(str(self.workpiece_path / (self.workpiece_name + '.obj')))
        
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
            baseInertialFramePosition=workpiece_geometric_center ,
        )

        # Find the geometric center of the surface
        surface_geometric_center = self.calculate_geometric_center(str(self.surface_path / (self.surface_name + '.obj')))

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
            basePosition=-surface_geometric_center,  # Move to make the geometric center the origin
            baseOrientation= [0, 0, 0, 1],  # first no rotation only translation to the center of the origin
            baseInertialFramePosition=surface_geometric_center ,
        )
        # Surface Rotation Operation
        surface_rotation = p.getQuaternionFromEuler([radians(self.Alpha), radians(self.Beta), 0])
    
        # Rotate the slide so that it is sloped and its cross section is rotated
        p.resetBasePositionAndOrientation(surface_id, [0,0,0], surface_rotation)
         
        # DEBUGGING VISUALIZATIONS
        # ----------------------------------------------------------------------------   

        # Enable visualizing collision shapes
        #p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1)

        # Enable the GUI and other visualizations if needed
        #p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

        # Draw a sphere at the adjusted COM to visualize it
        com_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1])  # Red sphere
        com_marker = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=com_visual_shape,
            basePosition=workpiece_geometric_center
        )
    
        # Update the COM marker position in each simulation step
        def update_com_marker():
            com_position, _ = p.getBasePositionAndOrientation(workpiece_id)
            p.resetBasePositionAndOrientation(com_marker, com_position, [0, 0, 0, 1])
        # ----------------------------------------------------------------------------

        # Set dynamic properties for the plane
        p.changeDynamics(surface_id, -1, restitution=0.8, lateralFriction=0.5, linearDamping=0.04, angularDamping=0.1)

        # Set dynamic properties for the workpiece mass is in kg/m³ and the density of Aqua 8K V4 Resin is 1100 kg/m³
        p.changeDynamics(workpiece_id, -1, mass=1, restitution=0.8, lateralFriction=0.5, linearDamping=0.04, angularDamping=0.1)

        # File paths for simulated data
        workpiece_data_path = self.data_path / (self.workpiece_name + '_simulated_data.txt')
        workpiece_location_path = self.data_path / (self.workpiece_name + '_simulated_data_export_matlab_location.txt')
        workpiece_rotation_path = self.data_path / (self.workpiece_name + '_simulated_data_export_matlab_rotation.txt')
        workpiece_quaternion_path = self.data_path / (self.workpiece_name + '_simulated_data_export_matlab_quaternion.txt')

        # Initialize matricies to store simulation data
        matrix_location = []
        matrix_rotation_euler = []
        matrix_rotation_quaternion = []

        # Clear the text files before starting (open in 'w' mode)
        with open(workpiece_location_path, 'w') as loc_file, \
            open(workpiece_rotation_path, 'w') as rot_file, \
            open(workpiece_quaternion_path, 'w') as quat_file:
            pass  # Just opening the files in 'w' mode will clear them

        # Open workpiece data file once
        with open(workpiece_data_path, 'w') as workpiece_data:
            workpiece_data.writelines("-------------------------------------------")
            workpiece_data.writelines('\nSimulated_Data_' + self.workpiece_name + '\n')
            workpiece_data.writelines("-------------------------------------------\n")

            for n in range(1, self.simulation_number + 1):

                # Random workpiece rotations defined in each iteration
                workpiece_start_orientation = self.rand_orientation()

                # Reset the position and orientation of the workpiece before each iteration
                p.resetBasePositionAndOrientation(workpiece_id, workpiece_start_pos, workpiece_start_orientation)
                p.resetBaseVelocity(workpiece_id, [0, 0, 0], [0, 0, 0])
                
                # Run the simulation
                for step in range(simulation_steps):  # Maximum number of simulation steps
                    p.stepSimulation()  # Step the simulation forward

                    update_com_marker()

                    # Get the workpiece's current position and orientation
                    position, bullet_orientation = p.getBasePositionAndOrientation(workpiece_id)

                    # Apply an orientation to negate Alpha and Beta
                    # Create a quaternion to negate Alpha and Beta rotations
                    negating_rotation = p.invertTransform([0, 0, 0], surface_rotation)[1] 

                    # Use PyBullet's multiplyTransforms to multiply quaternions
                    _, bullet_orientation = p.multiplyTransforms([0, 0, 0], negating_rotation, [0, 0, 0], bullet_orientation)

                    #Change quaternion ordering from w,x,y,z to x,y,z,w to match blender model outputs
                    blender_orientation = (bullet_orientation[1], bullet_orientation[2], bullet_orientation[3], bullet_orientation[0])

                    euler_orientation = p.getEulerFromQuaternion(bullet_orientation)

                    #Change euler orientation to match blender model outputs
                    euler_orientation =[degrees(euler_orientation[0])-90, degrees(euler_orientation[1]), degrees(euler_orientation[2])]
                    
                    # Get linear and angular velocity to detect stopping condition
                    linear_velocity, angular_velocity = p.getBaseVelocity(workpiece_id)
                    angular_velocity_magnitude = np.linalg.norm(angular_velocity)

                    # Get contact points between the plane and the workpiece
                    contact_points = p.getContactPoints(bodyA=surface_id, bodyB=workpiece_id)

                    # Check if angular velocity is below the threshold and there is contact
                    if angular_velocity_magnitude < impulse_threshold and len(contact_points) > 0:
                        print(f"Object '{self.workpiece_name}' reached equilibrium at step {step} with contact")
                        break

                    # Slow down the simulation to match real-time (optional)
                    time.sleep(1 / 240.)

                # Store data
                matrix_location.append(position)
                matrix_rotation_euler.append(euler_orientation)
                matrix_rotation_quaternion.append(blender_orientation)

                # Write iteration data to workpiece_data
                workpiece_data.writelines(f"\nITERATION: {n}\n")
                workpiece_data.writelines(f"LAST STEP: {step}\n")
                workpiece_data.writelines(f"Simulated Location (XYZ) [mm]: {position[0]}, {position[1]}, {position[2]}\n")
                workpiece_data.writelines(f"Simulated Rotation Euler (XYZ) [°]: {euler_orientation[0]}, {euler_orientation[1]}, {euler_orientation[2]}\n")
                workpiece_data.writelines(f"Simulated Rotation Quaternion (x, y, z, w): {bullet_orientation[0]}, {bullet_orientation[1]}, {bullet_orientation[2]}, {bullet_orientation[3]}\n")
            
            # Save the simulation data
            np.savetxt(workpiece_location_path, np.array(matrix_location), delimiter='\t')
            np.savetxt(workpiece_rotation_path, np.array(matrix_rotation_euler), delimiter='\t')
            np.savetxt(workpiece_quaternion_path, np.array(matrix_rotation_quaternion), delimiter='\t')


        # Disconnect from PyBullet
        p.disconnect()


