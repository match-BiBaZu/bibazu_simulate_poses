#----------
# 1. IMPORT
#----------
import bpy
import random
import math
import mathutils
from math import pi, degrees
import numpy as np

name_obj = "Teil_2"  # Object name
iterations = 100 # Number of iterations

previous_impulse = mathutils.Vector((0, 0, 0))  # Store the previous frame's impulse
previous_rotation = None # Store the previous frame's location

#File paths for imported stl files
surface_path = '/home/rosmatch/Dashas_fantastic_workspace/src/bibazu_simulate_poses/Surfaces/Slide_Long.STL'
workpiece_path = '/home/rosmatch/Dashas_fantastic_workspace/src/bibazu_simulate_poses/Workpieces/' + name_obj + '.STL'

# File paths for simulated data
base_path = '/home/rosmatch/Dashas_fantastic_workspace/src/bibazu_simulate_poses/SimulationData/Blender_Raw_Simulations/'
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
        # Deselect all objects
        bpy.ops.object.select_all(action='DESELECT')

        # Select all mesh objects
        bpy.ops.object.select_by_type(type='MESH')
        bpy.ops.object.delete()

        # Clear mesh data to avoid memory leaks
        for mesh in bpy.data.meshes:
            if mesh.users == 0:
                bpy.data.meshes.remove(mesh)
        # ------------------------------------------
        # 2. IMPORT, ADD, LOCATE and ROTATE Objects
        # ------------------------------------------

        # Import Plane and Workpiece
        bpy.ops.wm.stl_import(filepath=surface_path)
        plane = bpy.data.objects["Plane"]
        bpy.ops.wm.stl_import(filepath=workpiece_path)
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
        bpy.ops.object.origin_set(type='ORIGIN_CENTER_OF_MASS')
        bpy.ops.rigidbody.mass_calculate(material='Custom', density=1100) # Density of Aqua 8K V4 Resin (kg/m^3)
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
        # 6. Run Simulation and Export simulated data to .txt
        # -------------------------
                
        # Set scene frame range
        bpy.context.scene.frame_start = 1
        bpy.context.scene.frame_end = 2000

        # Set rigid body world frame range
        if bpy.context.scene.rigidbody_world:
            bpy.context.scene.rigidbody_world.point_cache.frame_start = bpy.context.scene.frame_start
            bpy.context.scene.rigidbody_world.point_cache.frame_end = bpy.context.scene.frame_end
            bpy.context.scene.rigidbody_world.point_cache.use_disk_cache = True

        # Initialize matrices to store location, rotation euler, and quaternion data
        matrix_location = np.zeros([2000, 3])
        matrix_rotation_euler = np.zeros([2000, 3])
        matrix_rotation_quaternion = np.zeros([2000, 4])
    
        # Simulate and capture data for each frame
        for f in range(bpy.context.scene.frame_start, bpy.context.scene.frame_end):
            bpy.context.scene.frame_set(f)

            o = bpy.data.objects["Workpiece"]
            vec_sim_loc = workpiece.matrix_world.to_translation()
            matrix_location[f] = [round(v, 4) for v in vec_sim_loc]

            simulated_rotation_rad = workpiece.matrix_world.to_euler()
            matrix_rotation_euler[f] = [round(math.degrees(v), 4) for v in simulated_rotation_rad]

            sim_quaternion = workpiece.matrix_world.to_quaternion()
            matrix_rotation_quaternion[f] = [round(v, 4) for v in sim_quaternion]

            if f > 100:
                if np.allclose(matrix_rotation_quaternion[f-8], matrix_rotation_quaternion[f], atol=1e-3) and \
                   np.allclose(matrix_rotation_quaternion[f-3], matrix_rotation_quaternion[f], atol=1e-3):
                    print(f"Object '{name_obj}' reached equilibrium at frame {f}")
                    break
            
            #if f > 5 and np.allclose(matrix_location[f], matrix_location[f - 5], atol=0.0001) and np.allclose(matrix_rotation_euler[f], matrix_rotation_euler[f - 5], atol=0.0001):
            #    break
                #print(f"Object '{name_obj}' reached equilibrium at frame {f}")
            #else:
                # If the loop completes without breaking, repeat the same iteration
            #    continue
        

        if f == bpy.context.scene.frame_end - 1:
            n -= 1  # Repeat simulation if the object in the test did not reach an equilibrium state
                    
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
