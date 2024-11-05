from pathlib import Path
import PoseFinder as pf
from PoseFinder import PoseFindingMode
import DroptestsFaster as dtf
#from STLtoOBJConverter import stl_to_obj_converter
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
simulation_number = 10

#--------------------------------------------------------------------------
# MODIFIABLE SURFACE AND WORKPIECE PARAMETERS:

alpha_array = np.arange(5, 45, 45) # degrees (set this to 90 when using the plane surface so that it is perpendicular to the gravity vector)

beta_array= np.arange(5, 45, 45) # degrees

workpiece_feed_speed_array = np.arange(0, 5, 5) # initial feed of the workpiece before it slides down the surface- mimics a conveyor belt feeder

hitpoint_offset_parallel_array = np.arange(0, 0.03, 0.03) # offset of the force application hitpoint on the workpiece from the geometric center of the workpiece parallel to the sliding axis

nozzle_offset_parallel = 0.5 # offset of the nozzle on one of the slide surfaces parallel to the sliding axis from the input end of the surface

nozzle_offset_perpendicular_array = np.arange(0, 0.06, 0.06) # offset of the nozzle on one of the slide surface perpendicular from the sliding axis

nozzle_impulse_force_array = np.arange(0, 5, 5) # impulse force applied by the nozzle to the workpiece

#Create an .obj file if it does not already exist for the bullet engine

#if not (workpiece_path / (workpiece_name + '.obj')).exists():
#stl_to_obj_converter(str(workpiece_path / (workpiece_name + '.STL')), str(workpiece_path / (workpiece_name + '.obj')),0.001)

#if not (surface_path / (surface_name + '.obj')).exists():
#stl_to_obj_converter(str(surface_path / (surface_name + '.STL')), str(surface_path / (surface_name + '.obj')),0.05)

# Loop through the modifiable parameters and generate simulation data for each combination of parameters
for alpha in alpha_array:
    for beta in beta_array:
        for workpiece_feed_speed in workpiece_feed_speed_array:
            for hitpoint_offset_parallel in hitpoint_offset_parallel_array:
                for nozzle_offset_perpendicular in nozzle_offset_perpendicular_array:
                    for nozzle_impulse_force in nozzle_impulse_force_array:

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
                            Alpha = alpha,
                            Beta = beta,
                            workpiece_feed_speed = workpiece_feed_speed,
                            hitpoint_offset_parallel = hitpoint_offset_parallel,
                            nozzle_offset_parallel = nozzle_offset_parallel,
                            nozzle_offset_perpendicular = nozzle_offset_perpendicular,
                            nozzle_impulse_force = nozzle_impulse_force,
                        )

                        # Generate the simulation data and write to csv files to store simulation data
                        drop_tests_simulator.drop_tests()



