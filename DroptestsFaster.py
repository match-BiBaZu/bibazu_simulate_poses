from pathlib import Path
import pybullet as p
import pybullet_data
import time
import random
from math import pi
import numpy as np # numpy HAS to be 1.26.4 at the latest for compatibility with PyBullet

class DroptestsFaster:
    def __init__(self):
        # This is the current name of the workpiece
        self.workpiece_name = 'r2d2'

        # This is the current name of the surface
        self.surface_name = 'Plane'

        # This is the total number of simulations - usually 1000
        self.simulation_number = 1000
        
        # This is the current file path of the data stored relative to the script
        self.data_path = Path(__file__).parent / 'SimulationData' / 'MHI_Data'

        # This is the current file path of the workpiece stls relative to the script
        self.workpiece_path =  Path(__file__).parent / 'Workpieces'

        # This is the current file path of the workpiece stls relative to the script
        self.surface_path =  Path(__file__).parent / 'Surfaces'

        # This is the current urdf file path of the workpiece
        self.workpiece_urdf_path = "r2d2.urdf"

        # THis is the current urdf file path of the surface
        self.surface_urdf_path = "plane.urdf"

    # Overrides the parameters defined in init, is done this way as you can have a flexible number of arguments
    def config(self, **kwargs):

        if 'workpiece_name' in kwargs:
            self.workpiece_name = kwargs['workpiece_name']
        if 'surface_name' in kwargs:
            self.workpiece_name = kwargs['surface_name']
        if 'data_path' in kwargs:
            self.data_path = kwargs['data_path']
        if 'workpiece_path' in kwargs:
            self.workpiece_path = kwargs['workpiece_path']
            self.workpiece_urdf_path = str(self.workpiece_path / (self.workpiece_name + '.urdf'))
        if 'surface_path' in kwargs:
            self.surface_path = kwargs['surface_path']
            self.surface_urdf_path = str(self.surface_path / (self.surface_name + '.urdf'))
        if 'simulation_number' in kwargs:
            self.simulation_number = kwargs['simulation_number']

    
    # Function to create a URDF file that points to an STL file
    @staticmethod
    def create_urdf( stl_path, urdf_path, object_name):
        urdf_content = f"""<?xml version="1.0"?>
        <robot name="{object_name}">
            <link name="base_link">
                <visual>
                    <geometry>
                        <mesh filename="{object_name}.STL" />
                    </geometry>
                </visual>
                <collision>
                    <geometry>
                        <mesh filename="{object_name}.STL" />
                    </geometry>
                </collision>
                <inertial>
                    <mass value="1.0" />
                    <origin xyz="0 0 0" rpy="0 0 0" />
                    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
                </inertial>
            </link>
        </robot>
        """
        # Write the URDF file
        with open(urdf_path, 'w') as urdf_file:
            urdf_file.write(urdf_content)
        print(f"URDF created: {urdf_path}")


    @staticmethod
    def rand_orientation():
        # Random rotations for workpiece
        rand_rot_x_W = round(random.uniform(0, 2 * pi), 5)
        rand_rot_y_W = round(random.uniform(0, 2 * pi), 5)
        rand_rot_z_W = round(random.uniform(0, 2 * pi), 5)

        workpiece_start_orientation = p.getQuaternionFromEuler([rand_rot_x_W , rand_rot_y_W, rand_rot_z_W])
        return workpiece_start_orientation
    
    def drop_tests(self):
        impulse_threshold = 0.001  # Define the impulse threshold for stopping
        simulation_steps = 2000 # Define the maximum number of simulation steps

        # Initialize PyBullet and set up physics simulation
        p.connect(p.GUI)  # Use p.DIRECT for non-GUI mode
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # PyBullet's internal data path
        #p.setAdditionalSearchPath("/home/rosmatch/Dashas_fantastic_workspace/src/bibazu_simulate_poses/")
        p.setGravity(0, 0, -9.81)  # Set gravity in the simulation

        # URDF file creation if file does not exist
        #if not (self.workpiece_path / (self.workpiece_name + '.urdf')).exists():
        self.create_urdf(str(self.workpiece_path / (self.workpiece_name + '.STL')), str(self.workpiece_path / (self.workpiece_name + '.urdf')), self.workpiece_name)
        #if not (self.surface_path / (self.surface_name + '.urdf')).exists():
        self.create_urdf(str(self.surface_path / (self.surface_name + '.STL')), str(self.surface_path / (self.surface_name + '.urdf')), self.surface_name)

        # Load the plane (ground)
        plane_id = p.loadURDF('plane.urdf')
        p.changeDynamics(plane_id, -1, restitution=0.8, lateralFriction=0.5)

        # Load the workpiece model (use a simple object for this example)
        workpiece_start_pos = [0, 0, 3]  # Start above the plane

        # workpiece_start_orientation = p.getQuaternionFromEuler([rand_rot_x_W , rand_rot_y_W, rand_rot_z_W])
        workpiece_start_orientation = self.rand_orientation()

        # Create the collision shape using the STL mesh
        collision_id = p.createCollisionShape(
            shapeType=p.GEOM_MESH,          # Specify that this is a mesh
            fileName= str(self.workpiece_path / 'Teil_1.obj'),             # Path to your STL file
            flags=p.URDF_INITIALIZE_SAT_FEATURES  # Optional, helps with complex shapes
        )

        # Create a visual shape (optional, for rendering in the GUI)
        vis_shape_id = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=str(self.workpiece_path / 'Teil_1.obj')
        )

        # Create a multi-body with the collision and visual shapes
        workpiece_id = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=collision_id,
            baseVisualShapeIndex=vis_shape_id,
            basePosition=workpiece_start_pos,
            baseOrientation=workpiece_start_orientation
        )

        #workpiece_id = p.loadURDF(self.workpiece_urdf_path, workpiece_start_pos, workpiece_start_orientation)

        # Set dynamic properties for the workpiece
        p.changeDynamics(workpiece_id, -1, restitution=0.8, lateralFriction=0.5, linearDamping=0.04, angularDamping=0.1)

        # File paths for simulated data
        workpiece_data_path = self.data_path / (self.workpiece_name + '_simulated_data.txt')
        workpiece_location_path = self.data_path / (self.workpiece_name + '_simulated_data_export_matlab_location.txt')
        workpiece_rotation_path = self.data_path / (self.workpiece_name + '_simulated_data_export_matlab_rotation.txt')
        workpiece_quaternion_path = self.data_path / (self.workpiece_name + '_simulated_data_export_matlab_quaternion.txt')

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

                matrix_location = []
                matrix_rotation_euler = []
                matrix_rotation_quaternion = []
                

                # Run the simulation
                for step in range(simulation_steps):  # Maximum number of simulation steps
                    p.stepSimulation()  # Step the simulation forward

                    # Get the workpiece's current position and orientation
                    pos, orn = p.getBasePositionAndOrientation(workpiece_id)
                    euler_orn = p.getEulerFromQuaternion(orn)

                    # Get linear and angular velocity to detect stopping condition
                    linear_velocity, angular_velocity = p.getBaseVelocity(workpiece_id)
                    angular_velocity_magnitude = np.linalg.norm(angular_velocity)  

                    # Get contact points between the plane and the workpiece
                    contact_points = p.getContactPoints(bodyA=plane_id, bodyB=workpiece_id)

                    # Check if angular velocity is below the threshold and there is contact
                    if angular_velocity_magnitude < impulse_threshold and len(contact_points) > 0:
                        print(f"Object '{self.workpiece_name}' reached equilibrium at step {step} with contact")
                        break

                    # Slow down the simulation to match real-time (optional)
                    #time.sleep(1 / 240.)

                # Store data
                matrix_location.append(pos)
                matrix_rotation_euler.append(euler_orn)
                matrix_rotation_quaternion.append(orn)

                # Save the simulation data
                np.savetxt(workpiece_location_path, np.array(matrix_location), delimiter='\t')
                np.savetxt(workpiece_rotation_path, np.array(matrix_rotation_euler), delimiter='\t')
                np.savetxt(workpiece_quaternion_path, np.array(matrix_rotation_quaternion), delimiter='\t')

                # Write iteration data to workpiece_data
                workpiece_data.writelines(f"\nITERATION: {n}\n")
                workpiece_data.writelines(f"LAST STEP: {step}\n")
                workpiece_data.writelines(f"Simulated Location (XYZ) [mm]: {pos[0]}, {pos[1]}, {pos[2]}\n")
                workpiece_data.writelines(f"Simulated Rotation Euler (XYZ) [°]: {euler_orn[0]}, {euler_orn[1]}, {euler_orn[2]}\n")
                workpiece_data.writelines(f"Simulated Rotation Quaternion (w, x, y, z): {orn[0]}, {orn[1]}, {orn[2]}, {orn[3]}\n")

        # Disconnect from PyBullet
        p.disconnect()


