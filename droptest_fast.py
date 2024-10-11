#----------
# 1. IMPORT
#----------

import bpy
import math
from math import pi, degrees, sin, cos
import numpy as np
import random
import gc

# Prepare .txt Export Files
name_obj = "Teil_5"  ########ÄNDERN#########

workpiece_data = open('/home/rosmatch/Dashas_fantastic_workspace/src/bibazu_simulate_poses/SimulationData/MHI_Data/'+name_obj+'_simulated_data.txt','w')
workpiece_data.writelines("-------------------------------------------")
workpiece_data.writelines('\n'+'Simulated_Data_'+name_obj+'\n')
workpiece_data.writelines("-------------------------------------------"+'\n')


workpiece_final_iterations_location = open('/home/rosmatch/Dashas_fantastic_workspace/src/bibazu_simulate_poses/SimulationData/MHI_Data/'+name_obj+'_simulated_data_export_matlab_location.txt','w')
workpiece_final_iterations_rotation = open('/home/rosmatch/Dashas_fantastic_workspace/src/bibazu_simulate_poses/SimulationData/MHI_Data/'+name_obj+'_simulated_data_export_matlab_rotation.txt','w')
workpiece_final_iterations_quaternion = open('/home/rosmatch/Dashas_fantastic_workspace/src/bibazu_simulate_poses/SimulationData/MHI_Data/'+name_obj+'_simulated_data_export_matlab_quaternion.txt','w')

iterations = 101

for n in range(1, iterations):

    #-------------------
    # 1.1 Delete Objects 
    #-------------------

    for i in bpy.context.scene.objects:
        if i.type == 'MESH':
            i.select_set(True)
        else:
            i.select_set(False)

    bpy.ops.object.delete()
    bpy.ops.object.select_all(action='DESELECT')

    #------------------------------------------
    # 2. IMPORT, ADD, LOCATE and ROTATE Objects
    #------------------------------------------

    # IMPORT STL Objects
    # IMPORT Plane
    filepath_STL_Plane = "/home/rosmatch/Dashas_fantastic_workspace/src/bibazu_simulate_poses/Surfaces/Plane.STL"
    print("print me to console")
    bpy.ops.wm.stl_import(filepath=filepath_STL_Plane) 
    
    # Rigid Body Constraints for Plane:
    # 1. Select Plane and make it PASSIVE Rigid Body
    bpy.data.objects["Plane"].select_set(True)
    plane = bpy.data.objects["Plane"]
    bpy.ops.rigidbody.objects_add(type='PASSIVE')
    bpy.ops.object.select_all(action='DESELECT')
    
    # 2. Surface Response Settings (Plane)
    bpy.data.objects["Plane"].select_set(True)
    
    bpy.context.object.rigid_body.restitution = 0.8  # Bounciness
    bpy.context.object.rigid_body.friction = 0.5     # Friction
    bpy.context.object.rigid_body.collision_shape = 'MESH'  # Collision Shape
    bpy.context.object.rigid_body.linear_damping = 0.04  # Translation Damping
    bpy.context.object.rigid_body.angular_damping = 0.1  # Rotation Damping
    
    bpy.ops.object.select_all(action='DESELECT')
    
    # IMPORT Workpiece
    filepath_STL_Workpiece = "/home/rosmatch/Dashas_fantastic_workspace/src/bibazu_simulate_poses/Workpieces/"+name_obj+".STL" ######ÄNDERN######
    bpy.ops.wm.stl_import(filepath=filepath_STL_Workpiece)
    
    # Rename Part
    bpy.data.objects[name_obj].select_set(True)  ######ÄNDERN######
    for obj in bpy.context.selected_objects:
        obj.name = "Workpiece"
    bpy.ops.object.select_all(action='DESELECT')
        
    # Rigid Body Constraints for Workpiece:
    # 1. Select Workpiece and make it an Active Rigid Body
    bpy.data.objects["Workpiece"].select_set(True)
    workpiece = bpy.data.objects["Workpiece"]  
    bpy.ops.rigidbody.objects_add(type='ACTIVE')
    density = 0.000989  # [g/mm^3]
    
    # Calculate VOLUME - DETERMINE MASS BASED ON DENSITY AND VOLUME
    
    bpy.context.object.rigid_body.mass = 1
    bpy.ops.object.select_all(action='DESELECT')
    
    # 2. Surface Response Settings (Workpiece)
    bpy.data.objects["Workpiece"].select_set(True)
    
    bpy.context.object.rigid_body.restitution = 0.8  # Bounciness
    bpy.context.object.rigid_body.friction = 0.5  # Friction
    bpy.context.object.rigid_body.collision_shape = 'CONVEX_HULL'  # Collision Shape
    bpy.context.object.rigid_body.linear_damping = 0.04  # Translation Damping
    bpy.context.object.rigid_body.angular_damping = 0.1  # Rotation Damping
    
    bpy.ops.object.select_all(action='DESELECT')

    # Select all MESH Objects
    for i in bpy.context.scene.objects:
        if i.type == 'MESH':
            i.select_set(True)
        else:
            i.select_set(False)

    # Set location WORKPIECE:
    vec_loc_W = np.array([0, 0, 60])  # Replacing mathutils.Vector with numpy array
    workpiece.location = vec_loc_W

    # Set location PLANE:
    vec_loc_P = np.array([0, 0, 0])
    plane.location = vec_loc_P

    # Set rotation PLANE using radians:
    vec_rot_P = [pi/2, 0, pi/2]  # Rotation in radians
    plane.rotation_euler = vec_rot_P

    # Set random rotation for Workpiece:
    rand_rot_x_W = round(random.uniform(0, 2 * pi), 5)
    rand_rot_y_W = round(random.uniform(0, 2 * pi), 5)
    rand_rot_z_W = round(random.uniform(0, 2 * pi), 5)

    vec_rot_W = [rand_rot_x_W, rand_rot_y_W, rand_rot_z_W]  # Rotation in radians
    workpiece.rotation_euler = vec_rot_W

    bpy.ops.object.select_all(action='DESELECT')

    #-------------------------
    # 3. RIGID BODY Constraints
    #-------------------------

    # 1. Select Workpiece and make it an Active Rigid Body
    bpy.data.objects["Workpiece"].select_set(True)
    bpy.ops.rigidbody.objects_add(type='ACTIVE')
    bpy.ops.object.select_all(action='DESELECT')

    # 2. Select Plane and make it PASSIVE Rigid Body
    bpy.data.objects["Plane"].select_set(True)
    bpy.ops.rigidbody.objects_add(type='PASSIVE')
    bpy.ops.object.select_all(action='DESELECT')

    #------------------------------------------------------------------------
    # 5. Simultaneously Printing of Object Location and Rotation to get POSE
    #------------------------------------------------------------------------


    def print_simulated_loc_rot(scene):
        C = bpy.context
        o = C.object

        simulated_loc = o.matrix_world.to_translation()  # Location as Vector
        simulated_rot_euler = o.matrix_world.to_euler()  # Euler Angles in Radians (XYZ)
        
        sim_rot_euler_degrees = [degrees(v) for v in simulated_rot_euler]  # Euler Angles in Degrees
        sim_rot_quaternion = o.matrix_world.to_quaternion()  # Quaternion 

        print_friendly_loc = [round(v, 5) for v in simulated_loc]
        print_friendly_rot_euler_degrees = [round(v, 5) for v in sim_rot_euler_degrees]
        print_friendly_rot_quaternion = [round(v, 5) for v in sim_rot_quaternion]

        workpiece_data.writelines("Location: ", print_friendly_loc)
        workpiece_data.writelines("Rotation EULER: ", print_friendly_rot_euler_degrees)
        workpiece_data.writelines("Rotation QUATERNION: ", print_friendly_rot_quaternion)
    
    bpy.app.handlers.frame_change_post.append(print_simulated_loc_rot)
    
    #----------------------------------------------------------------
    # 6. Export 'simulated location' and 'simulated rotation' as .txt
    #----------------------------------------------------------------

    workpiece = bpy.data.objects["Workpiece"]
    gc.collect()

    #---------------------------------------------------------------
    # Continue with exporting simulated locations and rotations...
    #---------------------------------------------------------------

workpiece_data.close()