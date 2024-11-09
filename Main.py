from pathlib import Path
import PoseFinder as pf
import DroptestsFaster as dtf
#from STLtoOBJConverter import stl_to_obj_converter
import numpy as np
from multiprocessing import Pool
import itertools

#--------------------------------------------------------------------------
# 1. Set up the paths to the stl files and the simulation data
#--------------------------------------------------------------------------

# Get the current script's directory
script_dir = Path(__file__).parent

# Get the current script's directory and then add the file path to the folders containing the position and orientation data of the stable poses recorded in the simulations
data_path = script_dir / 'Simulation_Data' / 'Bullet_Raw_Data' / 'Logged_Simulations'

# Get the current script's directory and then add the file path to the folders containing the workpiece stls
workpiece_path =  script_dir / 'Workpieces'

# Get the current script's directory and then add the file path to the folders containing the workpiece stls
surface_path =  script_dir / 'Surfaces'

#--------------------------------------------------------------------------
# 2. Create a DroptestsFaster instance to generate the simulation data
#--------------------------------------------------------------------------

# These are the names of the workpieces and of their models
workpiece_names = {'Teil_1','Teil_2','Teil_3','Teil_4','Teil_5'}

# This is the name of the surface used
surface_name = 'Slide_Long'

# This is the number of simulations
simulation_number = 100

#--------------------------------------------------------------------------
# MODIFIABLE SURFACE AND WORKPIECE PARAMETERS:

alpha_array = np.arange(0, 90, 5) # degrees (set this to 90 when using the plane surface so that it is perpendicular to the gravity vector)

beta_array = np.arange(0, 90, 5) # degrees

workpiece_feed_speed_array = np.arange(0, 6, 1) # initial feed of the workpiece before it slides down the surface- mimics a conveyor belt feeder

hitpoint_offset_parallel_array = np.arange(0, 0.035, 0.005) # offset of the force application hitpoint on the workpiece from the geometric center of the workpiece parallel to the sliding axis

nozzle_offset_parallel = 0.5 # offset of the nozzle on one of the slide surfaces parallel to the sliding axis from the input end of the surface

nozzle_offset_perpendicular_array = np.arange(0, 0.07, 0.01) # offset of the nozzle on one of the slide surface perpendicular from the sliding axis

nozzle_impulse_force_array = np.arange(0, 11, 1) # impulse force applied by the nozzle to the workpiece to reorient it

# Define the CSV file name with the workpiece name
csv_file_name = script_dir / 'Simulation_Data' / 'Bullet_Raw_Data' / ('simulation_outcomes.csv')

# Write the header line to the CSV file
with open(csv_file_name, 'w') as f:
    f.write(
        'workpiece,'
        'alpha,beta,workpiece_feed_speed,'
        'hitpoint_offset_parallel,'
        'nozzle_offset_perpendicular,'
        'nozzle_impulse_force,sliding_distance,'
        'Successfully_Re-oriented,'
        'Unsuccessfully_Re-oriented,'
        'Not_Reoriented,Not_Settled,'
        'Fell_Off_Slide\n'
    )

# Combine all parameter arrays for multiprocessing
parameter_combinations = list(itertools.product(
    workpiece_names,
    alpha_array,
    beta_array,
    workpiece_feed_speed_array,
    hitpoint_offset_parallel_array,
    nozzle_offset_perpendicular_array,
    nozzle_impulse_force_array
))

def run_simulation(params):
    workpiece_name,alpha, beta, workpiece_feed_speed, hitpoint_offset_parallel, nozzle_offset_perpendicular, nozzle_impulse_force = params

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
        mode = 1
    )

    # Generate the simulation data and write to csv files to store simulation data
    drop_tests_simulator.drop_tests()

    orientation_array = drop_tests_simulator.get_orientation_array()
    angular_velocity_array = drop_tests_simulator.get_angular_velocity_array()
    location_array = drop_tests_simulator.get_location_array()

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
        mode = 3
        )

    pose_finder.set_angular_velocity_array(angular_velocity_array)
    pose_finder.set_orientation_array(orientation_array)
    pose_finder.set_location_array(location_array)

    # Master function to find and plot the poses (if desired)
    pose_finder.find_poses_main()

    simulation_outcomes = pose_finder.get_simulation_outcome_frequency()
    sliding_distance = pose_finder.get_sliding_distance_average()

    # Append the input parameters and output parameters to a csv file
    with open(csv_file_name, 'a') as f:
        f.write(
            f"{workpiece_name},"
            f"{alpha},"
            f"{beta},"
            f"{workpiece_feed_speed},"
            f"{hitpoint_offset_parallel},"
            f"{nozzle_offset_perpendicular},"
            f"{nozzle_impulse_force},"
            f"{sliding_distance},"
            f"{simulation_outcomes[0]},"
            f"{simulation_outcomes[1]},"
            f"{simulation_outcomes[2]},"
            f"{simulation_outcomes[3]},"
            f"{simulation_outcomes[4]}\n"
        )

# Run simulations in parallel using 32 pools
if __name__ == "__main__":
    with Pool(32) as pool:
        pool.map(run_simulation, parameter_combinations)