from pathlib import Path
import PoseFinder as pf
from PoseFinder import PoseFindingMode
import DroptestsFaster as dtf
from STLtoOBJConverter import stl_to_obj_converter
import numpy as np

#--------------------------------------------------------------------------
# 1. Set up the paths to the stl files and the simulation data
#--------------------------------------------------------------------------

# Get the current script's directory
script_dir = Path(__file__).parent

# Get the current script's directory and then add the file path to the folders containing the position and orientation data of the stable poses recorded in the simulations
data_path = Path(__file__).parent / 'SimulationData'/'Dashas_Testing_Bullet'#'Dashas_Testing_Blender'

# Get the current script's directory and then add the file path to the folders containing the workpiece stls
workpiece_path =  Path(__file__).parent / 'Workpieces'

# Get the current script's directory and then add the file path to the folders containing the workpiece stls
surface_path =  Path(__file__).parent / 'Surfaces'

#--------------------------------------------------------------------------
# 2. Create a DroptestsFaster instance to generate the simulation data
#--------------------------------------------------------------------------

# This is the name of the workpieces and of their models
workpiece_name = 'Teil_4'

#This is the name of the surface used
surface_name = 'Slide_Long'

# This is the number of simulations
simulation_number = 100

#--------------------------------------------------------------------------
# MODIFIABLE SURFACE AND WORKPIECE PARAMETERS:

Alpha = 20.0 # degrees (set this to 90 when using the plane surface so that it is perpendicular to the gravity vector)

Beta = 5.0 # degrees

workpiece_feed_speed = 0.0001 # initial feed of the workpiece before it slides down the surface- mimics a conveyor belt feeder

hitpoint_offset_parallel = 0.002 # offset of the force application hitpoint on the workpiece from the geometric center of the workpiece parallel to the sliding axis

nozzle_offset_parallel = 0.1 # offset of the nozzle on one of the slide surfaces parallel to the sliding axis from the input end of the surface

nozzle_offset_perpendicular = 0.02 # offset of the nozzle on one of the slide surface perpendicular from the sliding axis

nozzle_impulse_force = 40.0 # impulse force applied by the nozzle to the workpiece

#Create an .obj file if it does not already exist for the bullet engine

#if not (workpiece_path / (workpiece_name + '.obj')).exists():
stl_to_obj_converter(str(workpiece_path / (workpiece_name + '.STL')), str(workpiece_path / (workpiece_name + '.obj')),0.001)

#if not (surface_path / (surface_name + '.obj')).exists():
stl_to_obj_converter(str(surface_path / (surface_name + '.STL')), str(surface_path / (surface_name + '.obj')),0.05)

# Create an instance to generate simulation data
drop_tests_simulator = dtf.DroptestsFaster()

# Define the workpiece name and filepath locations for the drop tests simulator instance
drop_tests_simulator.config(
    workpiece_name=workpiece_name,
    data_path=data_path,
    workpiece_path=workpiece_path,
    surface_path=surface_path,
    surface_name=surface_name,
    simulation_number=simulation_number,
    Alpha = Alpha,
    Beta = Beta,
    workpiece_feed_speed = workpiece_feed_speed,
    hitpoint_offset_parallel = hitpoint_offset_parallel,
    nozzle_offset_parallel = nozzle_offset_parallel,
    nozzle_offset_perpendicular = nozzle_offset_perpendicular,
    nozzle_impulse_force = nozzle_impulse_force,
)

# Generate the simulation data and write to csv files to store simulation data
drop_tests_simulator.drop_tests()

#--------------------------------------------------------------------------
# 3. Create a PoseFinder instance to process the simulation data
#--------------------------------------------------------------------------

# Create a pose finder instance to process the simulation data
pose_finder = pf.PoseFinder()

# Define the workpiece name and filepath locations for the pose finder instance
pose_finder.config(
    workpiece_name=workpiece_name,
    data_path=data_path,
    workpiece_path=workpiece_path,
    simulation_number=simulation_number,
    mode = PoseFindingMode.QUAT_COMPARE
    )

# Import csv data from simulations
pose_finder.import_orientation_csv()

# Master function to find and plot the poses
pose_finder.find_poses()

simulation_outcomes = pose_finder.get_simulation_outcomes()
sliding_distances = pose_finder.get_sliding_distance()

# Calculate the average of sliding distances
average_sliding_distance = np.mean(sliding_distances)
print('Sliding distances:', average_sliding_distance)

#--------------------------------------------------------------------------

# Process the data to find stable poses
#pose_finder.find_poses_quat() 

# Plot the resulting stable poses
#pose_finder.plot_poses_quat() # only got the quaternion outputs to output properly for pose finder so this only uses quaternions
