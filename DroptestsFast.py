#----------
# 1. IMPORT
#----------

import bpy
import random
import math
import mathutils
from math import pi, degrees
import numpy as np

name_obj = "Teil_1"  # Object name
iterations = 1000

# File paths
base_path = '/home/rosmatch/Dashas_fantastic_workspace/src/bibazu_simulate_poses/SimulationData/Dashas_Testing/'
workpiece_data_path = base_path + name_obj + '_simulated_data.txt'
workpiece_location_path = base_path + name_obj + '_simulated_data_export_matlab_location.txt'
workpiece_rotation_path = base_path + name_obj + '_simulated_data_export_matlab_rotation.txt'
workpiece_quaternion_path = base_path + name_obj + '_simulated_data_export_matlab_quaternion.txt'

# Clear the text files before starting (open in 'w' mode)
with open(workpiece_location_path, 'w') as loc_file, \
     open(workpiece_rotation_path, 'w') as rot_file, \
     open(workpiece_quaternion_path, 'w') as quat_file:
    pass  # Just opening the files in 'w' mode will clear them

# Open workpiece data file once
with open(workpiece_data_path, 'w') as workpiece_data:
    workpiece_data.writelines("-------------------------------------------")
    workpiece_data.writelines('\nSimulated_Data_' + name_obj + '\n')
    workpiece_data.writelines("-------------------------------------------\n")

    for n in range(1, iterations + 1):

        # -------------------
        # 1.1 Delete MESH Objects
        # -------------------
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.select_by_type(type='MESH')
        bpy.ops.object.delete()

        # ------------------------------------------
        # 2. IMPORT, ADD, LOCATE and ROTATE Objects
        # ------------------------------------------

        # Import Plane and Workpiece
        bpy.ops.wm.stl_import(filepath="/home/rosmatch/Dashas_fantastic_workspace/src/bibazu_simulate_poses/Surfaces/Plane.STL")
        plane = bpy.data.objects["Plane"]
        bpy.ops.wm.stl_import(filepath=f"/home/rosmatch/Dashas_fantastic_workspace/src/bibazu_simulate_poses/Workpieces/{name_obj}.STL")
        workpiece = bpy.data.objects[name_obj]
        workpiece.name = "Workpiece"  # Rename for consistency

        
        # Rigid Body Constraints for Plane
        plane.select_set(True)
        bpy.ops.rigidbody.objects_add(type='PASSIVE')
        bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY')
        plane.rigid_body.restitution = 0.8  # Bounciness
        plane.rigid_body.friction = 0.5  # Friction
        plane.rigid_body.collision_shape = 'MESH'  # Collision Shape
        plane.rigid_body.linear_damping = 0.04
        plane.rigid_body.angular_damping = 0.1
        
        # Set Plane location and rotation (fixed location)
        plane.location = mathutils.Vector((0, 0, 0))
        plane.rotation_euler = (pi/2,0,pi/2) #RADIANS
        
        bpy.ops.object.select_all(action='DESELECT')

        # Rigid Body Constraints for Workpiece
        workpiece.select_set(True)
        bpy.ops.rigidbody.objects_add(type='ACTIVE')
        bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY')
        workpiece.rigid_body.mass = 1
        workpiece.rigid_body.restitution = 0.8  # Bounciness
        workpiece.rigid_body.friction = 0.5  # Friction
        workpiece.rigid_body.collision_shape = 'CONVEX_HULL'
        workpiece.rigid_body.linear_damping = 0.04
        workpiece.rigid_body.angular_damping = 0.1

        # Random rotations for workpiece
        rand_rot_x_W = round(random.uniform(0, 2 * pi), 5)
        rand_rot_y_W = round(random.uniform(0, 2 * pi), 5)
        rand_rot_z_W = round(random.uniform(0, 2 * pi), 5)
        workpiece.rotation_euler = (rand_rot_x_W, rand_rot_y_W, rand_rot_z_W)

        # Fixed workpiece location (no randomization)
        workpiece.location = mathutils.Vector((0, 0, 60))
        
        bpy.ops.object.select_all(action='DESELECT')

        # -------------------------
        # 5. Simultaneously Printing of Object Location and Rotation to get POSE
        # -------------------------

        def print_simulated_loc_rot(scene):
            o = bpy.data.objects["Workpiece"]
            simulated_loc = o.matrix_world.to_translation()
            simulated_rot_euler = o.matrix_world.to_euler()  # EULER Angles in RADIANS (XYZ)
            sim_rot_euler_degrees = [degrees(v) for v in simulated_rot_euler]  # EULER Angles in DEGREES
            sim_rot_quaternion = o.matrix_world.to_quaternion()  # Quaternion 

            print_friendly_loc = [round(v, 5) for v in simulated_loc]
            print_friendly_rot_euler_degrees = [round(v, 5) for v in sim_rot_euler_degrees]
            print_friendly_rot_quaternion = [round(v, 5) for v in sim_rot_quaternion]

            print("Location: ", print_friendly_loc)
            print("Rotation EULER: ", print_friendly_rot_euler_degrees)
            print("Rotation QUATERNION: ", print_friendly_rot_quaternion)

        bpy.app.handlers.frame_change_post.clear()  # Clear previous handlers
        bpy.app.handlers.frame_change_post.append(print_simulated_loc_rot)

        # -------------------------
        # 6. Export simulated data to .txt
        # -------------------------

        bpy.context.scene.frame_end = 2000

        # Initialize matrices to store location, rotation euler, and quaternion data
        matrix_location = np.zeros([2000, 3])
        matrix_rotation_euler = np.zeros([2000, 3])
        matrix_rotation_quaternion = np.zeros([2000, 4])

        # Simulate and capture data for each frame
        for f in range(bpy.context.scene.frame_start, bpy.context.scene.frame_end):
            bpy.context.scene.frame_set(f)

            o = bpy.data.objects["Workpiece"]
            vec_sim_loc = o.matrix_world.to_translation()
            matrix_location[f] = [round(v, 4) for v in vec_sim_loc]

            simulated_rotation_rad = o.matrix_world.to_euler()
            matrix_rotation_euler[f] = [round(math.degrees(v), 4) for v in simulated_rotation_rad]

            sim_quaternion = o.matrix_world.to_quaternion()
            matrix_rotation_quaternion[f] = [round(v, 4) for v in sim_quaternion]

        # Write simulated data to files
        with open(workpiece_location_path, 'a') as loc_file, open(workpiece_rotation_path, 'a') as rot_file, open(workpiece_quaternion_path, 'a') as quat_file:
            loc_file.write(f"{matrix_location[f][0]}\t{matrix_location[f][1]}\t{matrix_location[f][2]}\n")
            rot_file.write(f"{matrix_rotation_euler[f][0]}\t{matrix_rotation_euler[f][1]}\t{matrix_rotation_euler[f][2]}\n")
            quat_file.write(f"{matrix_rotation_quaternion[f][0]}\t{matrix_rotation_quaternion[f][1]}\t{matrix_rotation_quaternion[f][2]}\t{matrix_rotation_quaternion[f][3]}\n")
        
        # Write iteration data to workpiece_data
        workpiece_data.writelines(f"\nITERATION: {n}\n")
        workpiece_data.writelines(f"LAST FRAME: {f}\n")
        workpiece_data.writelines(f"Simulated Location (XYZ) [mm]: {matrix_location[f][0]}, {matrix_location[f][1]}, {matrix_location[f][2]}\n")
        workpiece_data.writelines(f"Simulated Rotation Euler (XYZ) [°]: {matrix_rotation_euler[f][0]}, {matrix_rotation_euler[f][1]}, {matrix_rotation_euler[f][2]}\n")
        workpiece_data.writelines(f"Simulated Rotation Quaternion (w, x, y, z): {matrix_rotation_quaternion[f][0]}, {matrix_rotation_quaternion[f][1]}, {matrix_rotation_quaternion[f][2]}, {matrix_rotation_quaternion[f][3]}\n")
